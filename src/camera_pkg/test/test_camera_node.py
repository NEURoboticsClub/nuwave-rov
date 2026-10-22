### NOTE: This is a first draft written entirely by claude. I will be going through it, updating, changing, and revising accordingly after simulation headway has been made. 


import threading
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from builtin_interfaces.msg import Time

from camera_pkg import camera_functions
from camera_pkg.camera_functions import setup_camera, capture_loop, publish_frame


# ---------------------------------------------------------------------------
# Fakes
#
# These stand in for the camera, logger, clock and publisher, so the tests
# run without camera hardware or a running ROS graph.
# ---------------------------------------------------------------------------

class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class FakeClock:
    """Hands out stamps of 1 s, 2 s, 3 s, ... on successive now() calls."""

    def __init__(self):
        self.count = 0

    def now(self):
        self.count += 1
        stamp = Time(sec=self.count)
        return SimpleNamespace(to_msg=lambda: stamp)


class FakePublisher:
    def __init__(self, error=None):
        self.messages = []
        self.error = error

    def publish(self, msg):
        if self.error is not None:
            raise self.error
        self.messages.append(msg)


class FakeCapture:
    """Stands in for a cv2.VideoCapture.

    reads lists what successive read() calls do: a (ret, frame) tuple to
    return, or an exception to raise. Once the list runs out, read() sets
    stop_event (if one was given) and returns (False, None). That way a
    capture_loop under test always finishes on its own.
    """

    def __init__(self, opened=True, reads=None, stop_event=None, release_error=None):
        self.opened = opened
        self.reads = list(reads or [])
        self.stop_event = stop_event
        self.release_error = release_error
        self.read_count = 0
        self.released = False
        self.props = {}

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def release(self):
        self.released = True
        if self.release_error is not None:
            raise self.release_error

    def read(self):
        self.read_count += 1
        if not self.reads:
            if self.stop_event is not None:
                self.stop_event.set()
            return False, None
        result = self.reads.pop(0)
        if isinstance(result, Exception):
            raise result
        return result


def fail_if_reopened(old_cap):
    pytest.fail("capture_loop reopened the camera when it shouldn't have")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def no_sleep(monkeypatch):
    """Skip the retry delays so the tests run instantly."""
    monkeypatch.setattr(camera_functions.time, "sleep", lambda seconds: None)


@pytest.fixture
def logger():
    return FakeLogger()


@pytest.fixture
def fake_video_capture(monkeypatch):
    """Replace cv2.VideoCapture with a stub that hands out queued results in order.

    Call it with a list of FakeCaptures (or exceptions to raise). It returns a
    list that records the arguments of every cv2.VideoCapture(...) call.
    """
    def install(results):
        queue = list(results)
        calls = []

        def video_capture(*args):
            calls.append(args)
            if not queue:
                # pytest.fail is not caught by setup_camera's `except Exception`,
                # so this stops the retry loop instead of spinning forever.
                pytest.fail("setup_camera opened the camera more times than expected")
            result = queue.pop(0)
            if isinstance(result, Exception):
                raise result
            return result

        monkeypatch.setattr(camera_functions.cv2, "VideoCapture", video_capture)
        return calls

    return install


# ---------------------------------------------------------------------------
# setup_camera
# ---------------------------------------------------------------------------

def test_setup_camera_opens_and_configures(fake_video_capture, logger):
    cap = FakeCapture()
    calls = fake_video_capture([cap])

    result = setup_camera('/dev/video0', 320, 240, 30, logger)

    assert result is cap
    assert calls == [('/dev/video0', cv2.CAP_V4L2)]
    assert cap.props == {
        cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG'),
        cv2.CAP_PROP_FRAME_WIDTH: 320,
        cv2.CAP_PROP_FRAME_HEIGHT: 240,
        cv2.CAP_PROP_FPS: 30,
        cv2.CAP_PROP_BUFFERSIZE: 1,
    }
    assert logger.warnings == []


def test_setup_camera_retries_until_camera_opens(fake_video_capture, logger):
    failed_1 = FakeCapture(opened=False)
    failed_2 = FakeCapture(opened=False)
    good = FakeCapture()
    calls = fake_video_capture([failed_1, failed_2, good])

    result = setup_camera('/dev/video0', 320, 240, 30, logger)

    assert result is good
    assert len(calls) == 3
    assert failed_1.released and failed_2.released  # failed attempts get cleaned up
    assert not good.released


def test_setup_camera_releases_existing_capture(fake_video_capture, logger):
    old = FakeCapture()
    new = FakeCapture()
    fake_video_capture([new])

    result = setup_camera('/dev/video0', 320, 240, 30, logger, cap=old)

    assert old.released
    assert result is new


def test_setup_camera_logs_when_release_fails(fake_video_capture, logger):
    old = FakeCapture(release_error=RuntimeError("release failed"))
    new = FakeCapture()
    fake_video_capture([new])

    result = setup_camera('/dev/video0', 320, 240, 30, logger, cap=old)

    assert result is new
    assert len(logger.warnings) == 1
    assert "Could not release" in logger.warnings[0]


def test_setup_camera_logs_and_retries_when_open_raises(fake_video_capture, logger):
    good = FakeCapture()
    fake_video_capture([RuntimeError("device busy"), good])

    result = setup_camera('/dev/video0', 320, 240, 30, logger)

    assert result is good
    assert len(logger.warnings) == 1
    assert "device busy" in logger.warnings[0]


# ---------------------------------------------------------------------------
# capture_loop
#
# These call capture_loop directly instead of in a thread. The FakeCapture
# sets stop_event when it runs out of scripted reads, so the loop returns.
# ---------------------------------------------------------------------------

def test_capture_loop_delivers_frames_with_stamps(logger):
    stop_event = threading.Event()
    frame_a = np.zeros((4, 4, 3), dtype=np.uint8)
    frame_b = np.ones((4, 4, 3), dtype=np.uint8)
    cap = FakeCapture(reads=[(True, frame_a), (True, frame_b)], stop_event=stop_event)
    received = []

    capture_loop(cap, fail_if_reopened, lambda f, s: received.append((f, s)),
                 stop_event, FakeClock(), logger)

    assert len(received) == 2
    assert received[0][0] is frame_a
    assert received[1][0] is frame_b
    assert [stamp.sec for _, stamp in received] == [1, 2]
    assert logger.warnings == []


def test_capture_loop_tolerates_up_to_10_failures_in_a_row(logger):
    # Two runs of 10 failures. If the counter didn't reset after a good
    # frame, the second run would push it past 10 and trigger a reopen.
    stop_event = threading.Event()
    frame_a = np.zeros((4, 4, 3), dtype=np.uint8)
    frame_b = np.ones((4, 4, 3), dtype=np.uint8)
    reads = [(False, None)] * 10 + [(True, frame_a)] + [(False, None)] * 10 + [(True, frame_b)]
    cap = FakeCapture(reads=reads, stop_event=stop_event)
    received = []

    capture_loop(cap, fail_if_reopened, lambda f, s: received.append(f),
                 stop_event, FakeClock(), logger)

    assert len(received) == 2
    assert received[0] is frame_a
    assert received[1] is frame_b


def test_capture_loop_reopens_camera_after_more_than_10_failures(logger):
    stop_event = threading.Event()
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    dead_cap = FakeCapture(reads=[(False, None)] * 11, stop_event=stop_event)
    new_cap = FakeCapture(reads=[(True, frame)], stop_event=stop_event)
    reopened_with = []

    def reopen_camera(old_cap):
        reopened_with.append(old_cap)
        return new_cap

    received = []
    capture_loop(dead_cap, reopen_camera, lambda f, s: received.append(f),
                 stop_event, FakeClock(), logger)

    assert reopened_with == [dead_cap]
    assert len(received) == 1
    assert received[0] is frame  # frames now come from the reopened camera


def test_capture_loop_logs_read_exception_and_keeps_going(logger):
    stop_event = threading.Event()
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    cap = FakeCapture(reads=[RuntimeError("usb glitch"), (True, frame)], stop_event=stop_event)
    received = []

    capture_loop(cap, fail_if_reopened, lambda f, s: received.append(f),
                 stop_event, FakeClock(), logger)

    assert len(received) == 1
    assert received[0] is frame
    assert len(logger.warnings) == 1
    assert "usb glitch" in logger.warnings[0]


def test_capture_loop_does_not_read_when_already_stopped(logger):
    stop_event = threading.Event()
    stop_event.set()
    cap = FakeCapture(reads=[(True, np.zeros((4, 4, 3), dtype=np.uint8))])

    capture_loop(cap, fail_if_reopened, lambda f, s: None,
                 stop_event, FakeClock(), logger)

    assert cap.read_count == 0


def test_capture_loop_stops_when_event_set_from_another_thread(logger):
    # Mirrors how the node uses it: loop in a background thread, stopped from outside.
    class EndlessCapture(FakeCapture):
        def read(self):
            self.read_count += 1
            return True, np.zeros((4, 4, 3), dtype=np.uint8)

    stop_event = threading.Event()
    got_frame = threading.Event()
    thread = threading.Thread(
        target=capture_loop,
        args=(EndlessCapture(), fail_if_reopened, lambda f, s: got_frame.set(),
              stop_event, FakeClock(), logger),
        daemon=True,
    )
    thread.start()

    assert got_frame.wait(timeout=2.0)
    stop_event.set()
    thread.join(timeout=2.0)
    assert not thread.is_alive()


# ---------------------------------------------------------------------------
# publish_frame
# ---------------------------------------------------------------------------

def test_publish_frame_publishes_jpeg_message(logger):
    frame = np.full((240, 320, 3), 128, dtype=np.uint8)
    publisher = FakePublisher()

    publish_frame(frame, Time(sec=12, nanosec=34), publisher, 'camera_0', 70, logger)

    assert len(publisher.messages) == 1
    msg = publisher.messages[0]
    assert msg.format == 'jpeg'
    assert msg.header.frame_id == 'camera_0'
    assert msg.header.stamp.sec == 12
    assert msg.header.stamp.nanosec == 34

    # The data should decode back into (nearly) the same image.
    decoded = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (240, 320, 3)
    np.testing.assert_allclose(decoded.astype(float), frame.astype(float), atol=2)
    assert logger.warnings == []


def test_publish_frame_uses_jpeg_quality(logger):
    # Random noise compresses poorly, so the quality setting makes a big size difference.
    frame = np.random.default_rng(0).integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
    low = FakePublisher()
    high = FakePublisher()

    publish_frame(frame, Time(), low, 'camera_0', 10, logger)
    publish_frame(frame, Time(), high, 'camera_0', 95, logger)

    assert len(high.messages[0].data) > len(low.messages[0].data)


def test_publish_frame_skips_publish_when_encoding_fails(monkeypatch, logger):
    monkeypatch.setattr(camera_functions.cv2, "imencode", lambda ext, img, params: (False, None))
    publisher = FakePublisher()

    publish_frame(np.zeros((4, 4, 3), dtype=np.uint8), Time(), publisher, 'camera_0', 70, logger)

    assert publisher.messages == []
    assert logger.warnings == []


def test_publish_frame_logs_publish_error(logger):
    publisher = FakePublisher(error=RuntimeError("publisher gone"))

    publish_frame(np.zeros((8, 8, 3), dtype=np.uint8), Time(), publisher, 'camera_0', 70, logger)

    assert len(logger.warnings) == 1
    assert "publisher gone" in logger.warnings[0]


if __name__ == "__main__":
    pytest.main()