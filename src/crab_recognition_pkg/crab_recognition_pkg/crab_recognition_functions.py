"""Exercise the crab recognition node without a real YOLO model or live cameras."""

from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

GREEN_CRAB = 'green_crab'
OTHER_SPECIES = 'rock_crab'
CLASS_NAMES = {0: GREEN_CRAB, 1: OTHER_SPECIES}


class FakeBox:
    """Mimic one ultralytics detection box (cls, conf and xyxy are indexable)."""

    def __init__(self, class_id, confidence, xyxy=(10, 30, 60, 80)):
        self.cls = [class_id]
        self.conf = [confidence]
        self.xyxy = [list(xyxy)]


class FakeResults:
    """Mimic the first element of an ultralytics result list."""

    def __init__(self, boxes):
        self.boxes = boxes


def make_jpeg_message(width=64, height=48, value=128):
    """Build a CompressedImage holding a solid-colour JPEG."""
    frame = np.full((height, width, 3), value, dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', frame)
    assert ok
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = encoded.tobytes()
    return msg


def make_frame(width=64, height=48):
    """Build a plain BGR frame to place directly in the node's frame buffer."""
    return np.zeros((height, width, 3), dtype=np.uint8)


@pytest.fixture(scope='module')
def ros_context():
    """Init/shutdown rclpy once for the whole test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node_module():
    """Import the node module lazily, as the other package tests do."""
    import crab_recognition_pkg.crab_recognition_node as module
    return module


@pytest.fixture
def output_path(tmp_path):
    """Point the node at a not-yet-created folder inside the pytest temp dir."""
    return tmp_path / 'results' / 'result.jpg'


@pytest.fixture
def make_node(ros_context, node_module, output_path, monkeypatch):
    """Create nodes with YOLO replaced by a mock, and clean them up afterwards."""
    monkeypatch.setattr(node_module, 'DEFAULT_OUTPUT_PATH', str(output_path))
    created = []

    def factory(camera_count=None):
        if camera_count is not None:
            monkeypatch.setattr(node_module, 'DEFAULT_CAMERA_COUNT', camera_count)
        with patch.object(node_module, 'YOLO') as yolo_class:
            yolo_class.return_value = MagicMock()
            yolo_class.return_value.names = CLASS_NAMES
            node = node_module.CrabRecognitionNode()
        created.append(node)
        return node

    yield factory
    for node in created:
        node.destroy_node()


@pytest.fixture
def node(make_node):
    """Provide a node with the default four cameras."""
    return make_node()


def set_detections(node, boxes):
    """Make the mocked model return the given boxes for every frame."""
    node.model.return_value = [FakeResults(boxes)]


# --- construction and parameters -------------------------------------------------------


def test_default_parameters(node, node_module, output_path):
    """Load the defaults declared at the top of the module."""
    assert node.model_path == node_module.DEFAULT_MODEL_PATH
    assert node.output_path == str(output_path)
    assert node.confidence == pytest.approx(node_module.DEFAULT_CONFIDENCE)
    assert node.camera_topic_prefix == node_module.DEFAULT_IMAGE_TOPIC_PREFIX
    assert node.camera_count == node_module.DEFAULT_CAMERA_COUNT


def test_model_is_loaded_from_model_path(make_node, node_module):
    """Pass the configured weights path to YOLO exactly once."""
    with patch.object(node_module, 'YOLO') as yolo_class:
        created = node_module.CrabRecognitionNode()
    yolo_class.assert_called_once_with(node_module.DEFAULT_MODEL_PATH)
    created.destroy_node()


def test_starts_idle_with_no_frames(node):
    """Start with an empty frame buffer and no detection in progress."""
    assert node.latest_frames == {}
    assert node._busy is False


def test_one_subscription_per_camera(node):
    """Create one camera subscription for each configured camera."""
    assert len(node.camera_subs) == node.camera_count == 4


@pytest.mark.parametrize('requested, expected', [(0, 1), (-3, 1), (1, 1), (3, 3), (9, 4)])
def test_camera_count_is_clamped_to_one_through_four(make_node, requested, expected):
    """Clamp the camera count into the supported 1-4 range."""
    node = make_node(camera_count=requested)
    assert node.camera_count == expected
    assert len(node.camera_subs) == expected


def test_subscribes_to_expected_topics(node):
    """Listen on every camera topic and on the detection trigger topic."""
    camera_topics = [sub.topic_name for sub in node.camera_subs]
    assert camera_topics == [f'/camera_{i}/image/compressed' for i in range(4)]
    assert node.crab_scan_sub.topic_name == '/gui_buttons/detect_crabs'


# --- camera_callback -------------------------------------------------------------------


def test_camera_callback_stores_decoded_frame(node):
    """Decode a compressed image and keep it under its camera id."""
    node.camera_callback(make_jpeg_message(width=64, height=48), 2)
    assert set(node.latest_frames) == {2}
    assert node.latest_frames[2].shape == (48, 64, 3)
    assert node.latest_frames[2].dtype == np.uint8


def test_camera_callback_keeps_only_the_latest_frame(node):
    """Replace the earlier frame when the same camera publishes again."""
    node.camera_callback(make_jpeg_message(width=64, height=48), 0)
    node.camera_callback(make_jpeg_message(width=32, height=16), 0)
    assert node.latest_frames[0].shape == (16, 32, 3)


def test_camera_callback_buffers_cameras_independently(node):
    """Keep one frame per camera id."""
    node.camera_callback(make_jpeg_message(), 0)
    node.camera_callback(make_jpeg_message(), 3)
    assert set(node.latest_frames) == {0, 3}


def test_camera_callback_ignores_undecodable_data(node):
    """Drop corrupt image bytes instead of storing or raising."""
    msg = CompressedImage()
    msg.data = b'this is not a jpeg'
    node.camera_callback(msg, 1)
    assert node.latest_frames == {}


# --- crab_scan_callback ----------------------------------------------------------------


def test_scan_false_does_not_run_detection(node):
    """Ignore a trigger message whose data is False."""
    with patch.object(node, 'run_detection_once') as run:
        node.crab_scan_callback(Bool(data=False))
    run.assert_not_called()


def test_scan_true_runs_detection_once(node):
    """Run exactly one detection pass for a True trigger."""
    with patch.object(node, 'run_detection_once') as run:
        node.crab_scan_callback(Bool(data=True))
    run.assert_called_once_with()


def test_scan_ignored_while_busy(node):
    """Skip a trigger and warn if a detection pass is already running."""
    node._busy = True
    with patch.object(node, 'run_detection_once') as run, \
            patch.object(node, 'get_logger') as get_logger:
        node.crab_scan_callback(Bool(data=True))
    run.assert_not_called()
    get_logger.return_value.warning.assert_called_once()
    assert node._busy is True


def test_scan_sets_busy_flag_during_detection(node):
    """Hold the busy flag while the detection pass runs."""
    seen = []
    with patch.object(node, 'run_detection_once',
                      side_effect=lambda: seen.append(node._busy)):
        node.crab_scan_callback(Bool(data=True))
    assert seen == [True]
    assert node._busy is False


def test_scan_error_is_logged_and_busy_flag_cleared(node):
    """Log a failed detection pass and allow the next trigger to run."""
    with patch.object(node, 'run_detection_once', side_effect=RuntimeError('boom')), \
            patch.object(node, 'get_logger') as get_logger:
        node.crab_scan_callback(Bool(data=True))
    get_logger.return_value.error.assert_called_once()
    assert 'boom' in get_logger.return_value.error.call_args[0][0]
    assert node._busy is False

    with patch.object(node, 'run_detection_once') as run:
        node.crab_scan_callback(Bool(data=True))
    run.assert_called_once_with()


# --- run_detection_once ----------------------------------------------------------------


def test_detection_without_frames_skips_model_and_writes_nothing(node, output_path):
    """Warn and return when no camera frame has arrived yet."""
    with patch.object(node, 'get_logger') as get_logger:
        node.run_detection_once()
    node.model.assert_not_called()
    get_logger.return_value.warning.assert_called_once()
    assert list(output_path.parent.glob('*.jpg')) == []


def test_detection_creates_missing_output_directory(node, output_path):
    """Create the output folder even when no frames are available."""
    assert not output_path.parent.exists()
    node.run_detection_once()
    assert output_path.parent.is_dir()


def test_detection_writes_one_image_per_camera(node, output_path):
    """Save one annotated image for each camera that has a frame."""
    node.latest_frames = {0: make_frame(), 2: make_frame()}
    set_detections(node, [FakeBox(0, 0.9)])
    node.run_detection_once()

    files = sorted(path.name for path in output_path.parent.glob('*.jpg'))
    assert len(files) == 2
    assert files[0].startswith('result_') and files[0].endswith('_camera0.jpg')
    assert files[1].startswith('result_') and files[1].endswith('_camera2.jpg')
    assert node.model.call_count == 2


def test_detection_images_share_one_run_stamp(node, output_path):
    """Use the same timestamp for every camera in a single pass."""
    node.latest_frames = {0: make_frame(), 1: make_frame()}
    set_detections(node, [])
    node.run_detection_once()

    stamps = {'_'.join(path.stem.split('_')[1:4])
              for path in output_path.parent.glob('*.jpg')}
    assert len(stamps) == 1


def test_detection_saved_image_keeps_frame_size(node, output_path):
    """Write images that can be read back at the original resolution."""
    node.latest_frames = {0: make_frame(width=80, height=60)}
    set_detections(node, [FakeBox(0, 0.8, xyxy=(5, 10, 40, 50))])
    node.run_detection_once()

    saved = cv2.imread(str(next(output_path.parent.glob('*_camera0.jpg'))))
    assert saved.shape == (60, 80, 3)


def test_detection_passes_confidence_threshold_to_model(node):
    """Call the model with the configured confidence and quiet logging."""
    node.latest_frames = {0: make_frame()}
    set_detections(node, [])
    node.run_detection_once()

    kwargs = node.model.call_args.kwargs
    assert kwargs['conf'] == pytest.approx(node.confidence)
    assert kwargs['verbose'] is False


def test_detection_does_not_modify_buffered_frame(node):
    """Annotate a copy so the stored frame stays untouched."""
    frame = make_frame()
    node.latest_frames = {0: frame}
    set_detections(node, [FakeBox(0, 0.95, xyxy=(5, 5, 30, 30))])
    node.run_detection_once()

    assert node.latest_frames[0] is frame
    assert not frame.any()


def test_detection_only_draws_green_crabs(node, node_module):
    """Draw a box for green crabs and skip every other class."""
    node.latest_frames = {0: make_frame()}
    set_detections(node, [FakeBox(0, 0.9), FakeBox(1, 0.95), FakeBox(0, 0.7)])
    with patch.object(node_module.cv2, 'rectangle', wraps=cv2.rectangle) as rectangle:
        node.run_detection_once()
    assert rectangle.call_count == 2


def test_detection_with_no_boxes_still_saves_image(node, output_path, node_module):
    """Save an annotated frame even when nothing was detected."""
    node.latest_frames = {0: make_frame()}
    set_detections(node, [])
    with patch.object(node_module.cv2, 'rectangle', wraps=cv2.rectangle) as rectangle:
        node.run_detection_once()
    rectangle.assert_not_called()
    assert len(list(output_path.parent.glob('*_camera0.jpg'))) == 1


def test_detection_logs_per_camera_and_total_counts(node):
    """Report each camera's green crab count and the overall total."""
    node.latest_frames = {0: make_frame(), 1: make_frame()}
    set_detections(node, [FakeBox(0, 0.9), FakeBox(1, 0.9)])
    with patch.object(node, 'get_logger') as get_logger:
        node.run_detection_once()

    messages = [call.args[0] for call in get_logger.return_value.info.call_args_list]
    assert any('Camera 0 green crabs found: 1' in message for message in messages)
    assert any('Camera 1 green crabs found: 1' in message for message in messages)
    assert any('Total green crabs across 2 camera frames: 2' in message
               for message in messages)


def test_detection_keeps_extension_of_output_path(node_module, make_node, tmp_path,
                                                  monkeypatch):
    """Keep the configured extension in every per-camera filename."""
    png_path = tmp_path / 'pngs' / 'result.png'
    monkeypatch.setattr(node_module, 'DEFAULT_OUTPUT_PATH', str(png_path))
    node = make_node()
    node.latest_frames = {0: make_frame()}
    set_detections(node, [])
    node.run_detection_once()

    assert [path.suffix for path in png_path.parent.iterdir()] == ['.png']
