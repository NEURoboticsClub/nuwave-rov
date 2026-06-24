import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure, Temperature, CompressedImage, Imu
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import asyncio
import datetime
import json
import re
import threading
import aiohttp
import os
import cv2
import numpy as np
from aiohttp import web

# Screenshots are saved to the home directory and on any
# plugged-in removable drive
SCREENSHOT_HOME_DIR = os.path.expanduser('~/rov_images')
SCREENSHOT_SUBDIR = 'rov_images'
# Ubuntu auto-mounts removable drives under /media/<user>/<label> or /run/media/<user>/<label>
REMOVABLE_MEDIA_ROOTS = ['/media', '/run/media']

# "Take Video" button captures all cameras into rov_images/video/<timestamp>/cam_<id>/<frame>.png
VIDEO_SUBDIR = 'video'
VIDEO_FPS = 4


def removable_drive_dirs():
    """Return an <mount>/rov_images path for each mounted, writable USB drive."""
    user = os.environ.get('USER') or os.path.basename(os.path.expanduser('~'))
    dirs = []
    for root in REMOVABLE_MEDIA_ROOTS:
        user_root = os.path.join(root, user)
        if not os.path.isdir(user_root):
            continue
        for name in sorted(os.listdir(user_root)):
            mount = os.path.join(user_root, name)
            if os.path.ismount(mount) and os.access(mount, os.W_OK):
                dirs.append(os.path.join(mount, SCREENSHOT_SUBDIR))
    return dirs

# --- Shared state: connected WebSocket clients ---
ws_clients: set[web.WebSocketResponse] = set()
pending_messages: dict[str, object] = {}
# Latest compressed JPEG bytes per camera id, sent as binary
pending_frames: dict[int, bytes] = {}
pending_lock = threading.Lock()
flush_interval_s = 1.0 / 30.0

async def broadcast(data: dict):
    """Send JSON data to all connected browser clients."""
    msg = json.dumps(data)
    for ws in list(ws_clients):
        try:
            await ws.send_str(msg)
        except Exception:
            ws_clients.discard(ws)

async def broadcast_bytes(payload: bytes):
    """Send a raw binary frame to all connected browser clients."""
    for ws in list(ws_clients):
        try:
            await ws.send_bytes(payload)
        except Exception:
            ws_clients.discard(ws)

# --- ROS2 Node ---
class WebBridgeNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('web_bridge')
        self._loop = loop
        self._subs = []

        # Latest raw compressed frame bytes per camera id for screenshots
        self._latest_frames: dict[int, bytes] = {}
        self._frames_lock = threading.Lock()

        self._recording = False
        self._video_frame_index = 0
        self._video_session = None

        # Thruster telemetry topics
        thruster_topics = [
            '/thruster/thruster_fll',
            '/thruster/thruster_frl',
            '/thruster/thruster_rll',
            '/thruster/thruster_rrl',
            '/thruster/thruster_flv',
            '/thruster/thruster_frv',
            '/thruster/thruster_rlv',
            '/thruster/thruster_rrv',
        ]

        for topic in thruster_topics:
            self._subs.append(
                self.create_subscription(
                    Float32,
                    topic,
                    lambda msg, topic=topic: self._on_scalar(topic, msg),
                    10,
                )
            )

            response_topic = f'{topic}/response_pwm'
            self._subs.append(
                self.create_subscription(
                    Float32,
                    response_topic,
                    lambda msg, topic=response_topic: self._on_scalar(topic, msg),
                    10,
                )
            )

        # Power monitor telemetry: /power_monitor/monitor_<id>/<metric>
        metrics = ('bus_voltage', 'current', 'power', 'shunt_voltage')
        for monitor_id in range(8):
            for metric in metrics:
                topic = f'/power_monitor/monitor_{monitor_id}/{metric}'
                self._subs.append(
                    self.create_subscription(
                        Float32,
                        topic,
                        lambda msg, topic=topic: self._on_scalar(topic, msg),
                        10,
                    )
                )

        # Backward-compatible single-monitor topics remapped to monitor_0.
        for metric in metrics:
            topic = f'/power_monitor/{metric}'
            mapped_topic = f'/power_monitor/monitor_0/{metric}'
            self._subs.append(
                self.create_subscription(
                    Float32,
                    topic,
                    lambda msg, mapped_topic=mapped_topic: self._on_scalar(mapped_topic, msg),
                    10,
                )
            )

        # Arm motor telemetry topics
        arm_motor_topics = [
            '/arm/motor_base_yaw',
            '/arm/motor_base_pitch',
            '/arm/motor_elbow_pitch',
            '/arm/motor_wrist_yaw',
            '/arm/motor_wrist_pitch',
            '/arm/motor_claw',
        ]

        for topic in arm_motor_topics:
            self._subs.append(
                self.create_subscription(
                    Float32,
                    topic,
                    lambda msg, topic=topic: self._on_scalar(topic, msg),
                    10,
                )
            )

            response_topic = f'{topic}/response_pwm'
            self._subs.append(
                self.create_subscription(
                    Float32,
                    response_topic,
                    lambda msg, topic=response_topic: self._on_scalar(topic, msg),
                    10,
                )
            )

        # Depth telemetry
        self._subs.append(self.create_subscription(Float32, '/depth/depth', self._on_depth, 10))
        self._subs.append(self.create_subscription(FluidPressure, '/depth/pressure', self._on_pressure, 10))
        self._subs.append(self.create_subscription(Temperature, '/depth/temperature', self._on_temperature, 10))
        self._subs.append(self.create_subscription(Float32MultiArray, '/depth/depth_array', self._on_depth_array, 10))

        # IMU telemetry
        self._subs.append(self.create_subscription(Imu, '/imu', self._on_imu, 10))

        # Command topics
        self._subs.append(self.create_subscription(Twist, '/velocity_commands', self._on_velocity_commands, 10))
        self._subs.append(self.create_subscription(Float32MultiArray, '/arm_commands', self._on_arm_commands, 10))

        # Camera topics
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for camera_id in range(4):
            topic = f'/camera_{camera_id}/image/compressed'
            self._subs.append(
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, topic=topic: self._on_video(topic, msg),
                    camera_qos,
                )
            )

        # GUI button command publishers (browser -> ROS)
        self.detect_crabs_pub = self.create_publisher(Bool, '/gui_buttons/detect_crabs', 10)

        self.create_timer(1.0 / VIDEO_FPS, self._video_tick)

        self.get_logger().info('WebBridge node started')

    def handle_ws_message(self, raw_payload: str):
        """Handle a websocket message from the browser and forward supported topics to ROS."""
        try:
            payload = json.loads(raw_payload)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring non-JSON websocket payload')
            return

        topic = payload.get('topic')
        data = payload.get('data')
        if not isinstance(topic, str):
            self.get_logger().warning('Ignoring websocket payload with missing topic')
            return

        if topic == '/gui_buttons/detect_crabs':
            msg = Bool()
            msg.data = bool(data)
            self.detect_crabs_pub.publish(msg)
            return

        if topic == '/gui_buttons/screenshot':
            self._save_screenshots()
            return

        if topic == '/gui_buttons/take_video':
            self._set_recording(bool(data))
            return

        self.get_logger().debug(f'Ignoring unsupported websocket topic: {topic}')

    def _set_recording(self, enabled: bool):
        if enabled and not self._recording:
            self._video_session = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self._video_frame_index = 0
            self._recording = True
            self.get_logger().info(
                f'Video recording started ({VIDEO_FPS} fps) -> {VIDEO_SUBDIR}/{self._video_session}')
        elif not enabled and self._recording:
            self._recording = False
            self.get_logger().info(
                f'Video recording stopped after {self._video_frame_index} frames')

    def _video_tick(self):
        if not self._recording:
            return
        with self._frames_lock:
            frames = dict(self._latest_frames)
        if not frames:
            return

        index = self._video_frame_index
        session = self._video_session
        targets = [SCREENSHOT_HOME_DIR] + removable_drive_dirs()

        for camera_id, data in sorted(frames.items()):
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            for base in targets:
                try:
                    cam_dir = os.path.join(base, VIDEO_SUBDIR, session, f'cam_{camera_id}')
                    os.makedirs(cam_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(cam_dir, f'{index}.png'), frame)
                except OSError as exc:
                    self.get_logger().warning(f'Could not save video frame to {base}: {exc}')

        self._video_frame_index += 1

    def _save_screenshots(self):
        with self._frames_lock:
            frames = dict(self._latest_frames)
        if not frames:
            self.get_logger().warning('Screenshot requested but no camera frames yet')
            return

        timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        targets = [SCREENSHOT_HOME_DIR] + removable_drive_dirs()

        for camera_id, data in sorted(frames.items()):
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            for base in targets:
                # A failure on one target (e.g. a full or yanked USB stick)
                # must not stop the others from saving.
                try:
                    cam_dir = os.path.join(base, f'cam_{camera_id}')
                    os.makedirs(cam_dir, exist_ok=True)
                    path = os.path.join(cam_dir, f'cam_{camera_id}_{timestamp}.png')
                    if cv2.imwrite(path, frame):
                        self.get_logger().info(f'Saved screenshot: {path}')
                    else:
                        self.get_logger().warning(f'Failed to write screenshot: {path}')
                except OSError as exc:
                    self.get_logger().warning(f'Could not save to {base}: {exc}')

    def _forward(self, topic: str, payload):
        """Thread-safe: store the latest value for periodic websocket flush."""
        with pending_lock:
            pending_messages[topic] = payload

    async def _flush_loop(self):
        while True:
            await asyncio.sleep(flush_interval_s)
            with pending_lock:
                items = list(pending_messages.items())
                pending_messages.clear()
                frames = list(pending_frames.items())
                pending_frames.clear()

            for topic, payload in items:
                await broadcast({'topic': topic, 'data': payload})

            # Video frame binary: [camera_id byte][...JPEG bytes]
            for camera_id, data in frames:
                await broadcast_bytes(bytes([camera_id]) + data)

    def _on_scalar(self, topic: str, msg: Float32):
        self._forward(topic, float(msg.data))

    def _on_depth(self, msg: Float32):
        self._forward('/depth/depth', float(msg.data))

    def _on_pressure(self, msg: FluidPressure):
        self._forward('/depth/pressure', float(msg.fluid_pressure))

    def _on_temperature(self, msg: Temperature):
        self._forward('/depth/temperature', float(msg.temperature))

    def _on_depth_array(self, msg: Float32MultiArray):
        self._forward('/depth/depth_array', list(msg.data))

    def _on_imu(self, msg: Imu):
        payload = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w,
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
        }
        self._forward('/imu', payload)

    def _on_velocity_commands(self, msg: Twist):
        payload = {
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z,
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z,
            },
        }
        self._forward('/velocity_commands', payload)

    def _on_arm_commands(self, msg: Float32MultiArray):
        self._forward('/arm_commands', list(msg.data))

    def _on_video(self, topic: str, msg: CompressedImage):
        match = re.search(r'/camera_(\d+)/', topic)
        if not match:
            return
        camera_id = int(match.group(1))
        # One copy of the raw JPEG, shared by the binary stream and screenshots
        data = bytes(msg.data)
        with pending_lock:
            pending_frames[camera_id] = data
        with self._frames_lock:
            self._latest_frames[camera_id] = data

# --- HTTP + WebSocket server ---
async def ws_handler(request):
    node = request.app['ros_node']
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    ws_clients.add(ws)
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                node.handle_ws_message(msg.data)
    finally:
        ws_clients.discard(ws)
    return ws

def build_app(node: WebBridgeNode):
    app = web.Application()
    app['ros_node'] = node
    app.router.add_get('/ws', ws_handler)

    static_dir = os.path.join(get_package_share_directory('web_gui'), 'static')

    async def index(request):
        return web.FileResponse(os.path.join(static_dir, 'index.html'))

    app.router.add_get('/', index)
    app.router.add_static('/static', path=static_dir, name='static')
    return app

# run both ROS2 + web server
def main():
    rclpy.init()
    loop = asyncio.new_event_loop()

    node = WebBridgeNode(loop)

    # Spin ROS2 in a background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Run the web server on the main thread's asyncio loop
    app = build_app(node)
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, 'localhost', 8080)
    loop.run_until_complete(site.start())
    loop.create_task(node._flush_loop())
    node.get_logger().info('Web UI at http://localhost:8080')

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
