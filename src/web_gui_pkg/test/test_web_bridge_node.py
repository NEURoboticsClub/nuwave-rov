"""Test real WebSocket/HTTP, telemetry, queues, and image output with fake ROS."""

import asyncio
import copy
import importlib.util
import json
import queue
import sys
import threading
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import AsyncMock, Mock

import pytest


@pytest.fixture
def bridge(monkeypatch, tmp_path):
    """Run production networking and image code without ROS or background writers."""
    class Node:
        def __init__(self, name):
            self.name, self.logger = name, Mock()
            self.publishers, self.subscriptions, self.timers = [], [], []
            self.destroyed = False

        def create_publisher(self, message_type, topic, qos):
            pub = Mock(topic=topic, messages=[])
            pub.publish.side_effect = lambda msg: pub.messages.append(copy.deepcopy(msg))
            self.publishers.append(pub)
            return pub

        def create_subscription(self, message_type, topic, callback, qos):
            sub = NS(topic=topic, callback=callback, qos=qos)
            self.subscriptions.append(sub)
            return sub

        def create_timer(self, period, callback):
            timer = NS(period=period, callback=callback)
            self.timers.append(timer)
            return timer

        def get_logger(self):
            return self.logger

        def destroy_node(self):
            self.destroyed = True

    def install(name, **attributes):
        item = ModuleType(name)
        item.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, item)
        return item

    ros = install('rclpy', init=Mock(), shutdown=Mock())
    install('rclpy.node', Node=Node)
    executor = Mock()
    install('rclpy.executors', MultiThreadedExecutor=Mock(return_value=executor))
    policies = NS(RELIABLE=2, BEST_EFFORT=1, KEEP_LAST=1, TRANSIENT_LOCAL=1)
    install('rclpy.qos', QoSProfile=NS, ReliabilityPolicy=policies,
            HistoryPolicy=policies, DurabilityPolicy=policies)
    for package in ['std_msgs', 'geometry_msgs', 'sensor_msgs', 'ament_index_python']:
        install(package)
    install('std_msgs.msg', Float32=NS, Float32MultiArray=NS, Bool=NS)
    install('geometry_msgs.msg', Twist=NS)
    install('sensor_msgs.msg', FluidPressure=NS, Temperature=NS, CompressedImage=NS, Imu=NS)
    install('ament_index_python.packages', get_package_share_directory=lambda package:
            str(Path(__file__).resolve().parents[3] / 'src' / package))
    path = Path(__file__).parents[1] / 'web_gui_pkg' / 'web_bridge_node.py'
    spec = importlib.util.spec_from_file_location('_web_bridge_tests', path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'threading', NS(Thread=Mock(), Lock=threading.Lock))
    monkeypatch.setattr(module, 'SCREENSHOT_HOME_DIR', str(tmp_path / 'home'))
    monkeypatch.setattr(module, 'REMOVABLE_MEDIA_ROOTS', [])
    module.node = module.WebBridgeNode(None)
    module.ros, module.executor, module.tmp = ros, executor, tmp_path
    ok, encoded = module.cv2.imencode('.jpg', module.np.full((24, 32, 3), 120, 'uint8'))
    assert ok
    module.jpeg = encoded.tobytes()
    return module


def test_constructor_and_subscription_closures(bridge):
    node = bridge.node
    assert node.name == 'web_bridge' and len(node.subscriptions) == 78
    assert len(node.publishers) == 4
    assert node.timers[0].period == pytest.approx(0.25)
    assert node.timers[0].callback == node._video_tick
    assert node._video_queue.maxsize == bridge.VIDEO_QUEUE_MAX
    for topic in ['/thruster/thruster_fll', '/arm/motor_claw/response_pwm',
                  '/power_monitor/monitor_7/current', '/power_monitor/bus_voltage']:
        sub = next(sub for sub in node.subscriptions if sub.topic == topic)
        sub.callback(NS(data=2.5))
        mapped = '/power_monitor/monitor_0/bus_voltage' if topic.endswith('monitor/bus_voltage') else topic
        assert bridge.pending_messages[mapped] == 2.5
    modes = [sub for sub in node.subscriptions if sub.topic.startswith('/controls/')]
    assert all(sub.qos.durability == 1 and sub.qos.depth == 1 for sub in modes)


@pytest.mark.parametrize('raw', ['bad json', '[]', 'null', '0', '"text"', '{}', '{"topic":1}'])
def test_malformed_ws_payloads_are_ignored(bridge, raw):
    bridge.node.handle_ws_message(raw)
    assert all(not pub.messages for pub in bridge.node.publishers)


@pytest.mark.parametrize('topic, pub', [('detect_crabs', 'detect_crabs_pub'),
    ('expo_enabled', 'expo_toggle_pub'), ('precision_mode', 'precision_mode_toggle_pub'),
    ('stabilize_enabled', 'stabilize_toggle_pub')])
def test_ws_boolean_commands_publish_to_ros(bridge, topic, pub):
    for value in [True, False]:
        bridge.node.handle_ws_message(json.dumps({'topic': '/gui_buttons/' + topic, 'data': value}))
        assert getattr(bridge.node, pub).messages[-1].data is value


@pytest.mark.parametrize('topic, method, value', [('screenshot', '_save_screenshots', None),
    ('take_video', '_set_recording', True), ('camera_record', '_set_camera_record', {'camera': 2, 'enabled': False})])
def test_ws_media_commands_dispatch(bridge, monkeypatch, topic, method, value):
    target = Mock()
    monkeypatch.setattr(bridge.node, method, target)
    bridge.node.handle_ws_message(json.dumps({'topic': '/gui_buttons/' + topic, 'data': value}))
    assert target.call_args.args == (() if topic == 'screenshot' else (value,))
    bridge.node.handle_ws_message('{"topic":"/unknown"}')


@pytest.mark.parametrize('method, topic, field, value', [
    ('_on_depth', '/depth/depth', 'data', 2.5), ('_on_pressure', '/depth/pressure', 'fluid_pressure', 123400),
    ('_on_temperature', '/depth/temperature', 'temperature', 18.5),
    ('_on_depth_array', '/depth/depth_array', 'data', [1, 2]), ('_on_arm_commands', '/arm_commands', 'data', [0] * 6)])
def test_scalar_and_array_telemetry(bridge, method, topic, field, value):
    getattr(bridge.node, method)(NS(**{field: value}))
    assert bridge.pending_messages[topic] == value


def test_imu_twist_and_control_state_snapshots(bridge):
    vector = NS(x=1, y=2, z=3)
    orientation = NS(x=0, y=0, z=0, w=1)
    bridge.node._on_imu(NS(orientation=orientation, angular_velocity=vector, linear_acceleration=vector))
    assert bridge.pending_messages['/imu'] == {'orientation': vars(orientation),
        'angular_velocity': vars(vector), 'linear_acceleration': vars(vector)}
    bridge.node._on_velocity_commands(NS(linear=vector, angular=vector))
    assert bridge.pending_messages['/velocity_commands'] == {'linear': vars(vector), 'angular': vars(vector)}
    for mode in ['expo_enabled', 'precision_mode', 'stabilize_enabled']:
        getattr(bridge.node, '_on_' + mode)(NS(data=True))
        assert bridge.pending_messages['/controls/' + mode] is True
    snapshot = bridge.node.get_control_state_snapshot()
    snapshot.clear()
    assert len(bridge.node.get_control_state_snapshot()) == 3


def test_video_callback_coalesces_frames_and_ignores_unrecognized_topics(bridge):
    bridge.node._on_video('/invalid', NS(data=bridge.jpeg))
    assert not bridge.pending_frames
    for data in [b'old', bridge.jpeg]:
        bridge.node._on_video('/camera_2/image/compressed', NS(data=data))
    assert bridge.pending_frames == bridge.node._latest_frames == {2: bridge.jpeg}


@pytest.mark.parametrize('data', [None, [], {}, {'camera': 'bad'}])
def test_malformed_camera_record_controls_are_ignored(bridge, data):
    bridge.node._set_camera_record(data)
    assert bridge.node._disabled_cameras == set()


def test_recording_controls_queue_order_drop_handling_and_transitions(bridge):
    node = bridge.node
    node._video_tick()
    node._set_recording(True)
    session = node._video_session
    node._video_tick()
    assert node._video_frame_index == 0
    node._latest_frames = {3: bridge.jpeg, 1: bridge.jpeg, 0: bridge.jpeg}
    node._set_camera_record({'camera': 1, 'enabled': False})
    node._video_tick()
    assert [node._video_queue.get_nowait(), node._video_queue.get_nowait()] == [
        (session, 0, 0, bridge.jpeg), (session, 0, 3, bridge.jpeg)]
    node._set_recording(True)
    assert node._video_frame_index == 1
    node._set_camera_record({'camera': 1, 'enabled': True})
    node._video_queue = queue.Queue(maxsize=1)
    node._video_tick()
    node._video_tick()
    assert node._video_drops == 5 and node._video_queue.qsize() == 1
    assert node.logger.warning.call_count == 2
    node._set_recording(False)
    node._set_recording(False)
    assert not node._recording
    node._set_recording(True)
    assert node._video_frame_index == 0


def test_screenshot_requests_snapshot_frames_and_defer_io(bridge):
    node = bridge.node
    node._save_screenshots()
    node.logger.warning.assert_called_once()
    node._latest_frames = {1: bridge.jpeg}
    node._save_screenshots()
    kwargs = bridge.threading.Thread.call_args.kwargs
    assert kwargs['target'] == node._write_screenshots and kwargs['daemon']
    node._latest_frames[1] = b'new'
    assert kwargs['args'][0] == {1: bridge.jpeg}


@pytest.mark.parametrize('kind', ['screenshot', 'video'])
def test_image_workers_write_decodable_files_and_survive_bad_frames(bridge, kind):
    node = bridge.node
    if kind == 'screenshot':
        node._write_screenshots({2: bridge.jpeg, 3: b'bad JPEG'}, 'stamp')
        target = Path(bridge.SCREENSHOT_HOME_DIR) / 'cam2' / 'cam2_stamp.png'
    else:
        node._video_queue = NS(get=Mock(side_effect=[('session', 4, 2, bridge.jpeg),
            ('session', 5, 2, b'bad JPEG'), ('session', 6, 2, b''), KeyboardInterrupt()]))
        with pytest.raises(KeyboardInterrupt):
            node._video_writer_loop()
        target = Path(bridge.SCREENSHOT_HOME_DIR) / 'video/session/cam2/cam2_4.png'
        node.logger.error.assert_called_once()
    assert bridge.cv2.imread(str(target)).shape == (24, 32, 3)


@pytest.mark.parametrize('kind', ['screenshot', 'video'])
def test_one_failed_output_target_does_not_stop_other_targets(bridge, monkeypatch, kind):
    node = bridge.node
    original = bridge.os.makedirs
    monkeypatch.setattr(bridge, 'removable_drive_dirs', lambda: [str(bridge.tmp / 'usb')])

    def make_dirs(path, **kwargs):
        if str(path).startswith(bridge.SCREENSHOT_HOME_DIR):
            raise OSError('disk unavailable')
        return original(path, **kwargs)

    monkeypatch.setattr(bridge.os, 'makedirs', make_dirs)
    if kind == 'screenshot':
        node._write_screenshots({0: bridge.jpeg}, 'stamp')
        target = bridge.tmp / 'usb/cam0/cam0_stamp.png'
    else:
        node._video_queue = NS(get=Mock(side_effect=[('session', 0, 0, bridge.jpeg), KeyboardInterrupt()]))
        with pytest.raises(KeyboardInterrupt):
            node._video_writer_loop()
        target = bridge.tmp / 'usb/video/session/cam0/cam0_0.png'
    assert target.is_file()
    node.logger.warning.assert_called_once()


def test_failed_screenshot_write_is_reported(bridge, monkeypatch):
    monkeypatch.setattr(bridge.cv2, 'imwrite', Mock(return_value=False))
    bridge.node._write_screenshots({0: bridge.jpeg}, 'stamp')
    bridge.node.logger.warning.assert_called_once()


def test_removable_media_filters_and_sorts_mounted_writable_targets(bridge, monkeypatch):
    root = bridge.tmp / 'media'
    for label in ['Zed', 'Alpha', 'unmounted', 'readonly']:
        (root / 'pilot' / label).mkdir(parents=True)
    monkeypatch.setenv('USER', 'pilot')
    monkeypatch.setattr(bridge, 'REMOVABLE_MEDIA_ROOTS', [str(root), str(bridge.tmp / 'absent')])
    monkeypatch.setattr(bridge.os.path, 'ismount', lambda path: Path(path).name != 'unmounted')
    monkeypatch.setattr(bridge.os, 'access', lambda path, mode: Path(path).name != 'readonly')
    assert bridge.removable_drive_dirs() == [str(root / 'pilot' / label / 'rov_images') for label in ['Alpha', 'Zed']]


def test_client_coalesces_topics_and_frames_then_sends_wire_format(bridge):
    async def scenario():
        ws = NS(send_str=AsyncMock(), send_bytes=AsyncMock())
        client = bridge.Client(ws)
        ws.send_bytes.side_effect = lambda data: setattr(client, 'alive', False)
        bridge.clients.add(client)
        client.queue([('/depth', 1), ('/depth', 2)], [(2, b'old'), (2, b'JPEG')])
        await asyncio.wait_for(client.run(), 0.5)
        assert json.loads(ws.send_str.call_args.args[0]) == {'topic': '/depth', 'data': 2}
        ws.send_bytes.assert_awaited_once_with(b'\x02JPEG')
        assert not client.alive and client not in bridge.clients
    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['exception', 'timeout'])
def test_client_send_failure_or_timeout_removes_connection(bridge, failure):
    async def scenario():
        async def send(data):
            if failure == 'exception':
                raise OSError('disconnected')
            await asyncio.Event().wait()
        bridge.send_timeout_s = 0.01
        client = bridge.Client(NS(send_str=send))
        bridge.clients.add(client)
        client.queue([('/test', 1)], [])
        await asyncio.wait_for(client.run(), 0.5)
        assert not client.alive and client not in bridge.clients
    asyncio.run(scenario())


def test_flush_loop_recovers_from_client_error_and_delivers_next_batch(bridge):
    async def scenario():
        failed, delivered = asyncio.Event(), asyncio.Event()
        bridge.node.logger.error.side_effect = lambda text: failed.set()
        client = NS(queue=Mock(side_effect=[RuntimeError('bad client'), None]))
        # Client objects must be hashable like the production Client instances.
        client = Mock(queue=client.queue)
        bridge.clients.add(client)
        bridge.flush_interval_s = 0.001
        bridge.node._forward('/test', 'first')
        task = asyncio.create_task(bridge.node._flush_loop())
        try:
            await asyncio.wait_for(failed.wait(), 0.5)
            client.queue.side_effect = lambda messages, frames: delivered.set()
            bridge.node._forward('/test', 'old')
            bridge.node._forward('/test', 'latest')
            bridge.pending_frames[3] = b'JPEG'
            await asyncio.wait_for(delivered.wait(), 0.5)
            assert client.queue.call_args.args == ([('/test', 'latest')], [(3, b'JPEG')])
            assert not bridge.pending_messages and not bridge.pending_frames
        finally:
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
    asyncio.run(scenario())


def test_real_http_and_websocket_round_trip(bridge):
    from aiohttp.test_utils import TestClient, TestServer

    async def scenario():
        client = TestClient(TestServer(bridge.build_app(bridge.node)))
        await client.start_server()
        flush = asyncio.create_task(bridge.node._flush_loop())
        try:
            response = await client.get('/')
            assert response.status == 200 and '<html' in (await response.text()).lower()
            ws = await client.ws_connect('/ws')
            states = [await ws.receive_json(timeout=1) for _ in range(3)]
            assert {item['topic']: item['data'] for item in states} == bridge.node.get_control_state_snapshot()
            received = asyncio.Event()
            bridge.node.expo_toggle_pub.publish.side_effect = lambda msg: received.set()
            await ws.send_json({'topic': '/gui_buttons/expo_enabled', 'data': True})
            await asyncio.wait_for(received.wait(), 1)
            assert bridge.node.expo_toggle_pub.publish.call_args.args[0].data is True
            bridge.node._on_depth(NS(data=4.5))
            assert await ws.receive_json(timeout=1) == {'topic': '/depth/depth', 'data': 4.5}
            bridge.node._on_video('/camera_2/image/compressed', NS(data=bridge.jpeg))
            assert (await ws.receive(timeout=1)).data == b'\x02' + bridge.jpeg
            await ws.close()
        finally:
            flush.cancel()
            with pytest.raises(asyncio.CancelledError):
                await flush
            await client.close()
        assert not bridge.clients
    asyncio.run(scenario())


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), RuntimeError('loop failed')])
def test_main_cleanup(bridge, monkeypatch, error):
    loop = Mock()
    loop.run_forever.side_effect = error
    loop.create_task.side_effect = lambda coroutine: coroutine.close()
    monkeypatch.setattr(bridge.asyncio, 'new_event_loop', Mock(return_value=loop))
    monkeypatch.setattr(bridge, 'WebBridgeNode', Mock(return_value=bridge.node))
    monkeypatch.setattr(bridge, 'build_app', Mock(return_value=object()))
    monkeypatch.setattr(bridge, 'web', NS(AppRunner=Mock(), TCPSite=Mock()))
    if isinstance(error, RuntimeError):
        with pytest.raises(RuntimeError, match='loop failed'):
            bridge.main()
    else:
        bridge.main()
    bridge.executor.add_node.assert_called_once_with(bridge.node)
    assert bridge.node.destroyed
    bridge.ros.shutdown.assert_called_once_with()
