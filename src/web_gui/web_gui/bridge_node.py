import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure, Temperature, CompressedImage, Imu
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import asyncio
import json
import threading
import aiohttp
import os
from aiohttp import web

# --- Shared state: connected WebSocket clients ---
ws_clients: set[web.WebSocketResponse] = set()
pending_messages: dict[str, object] = {}
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

# --- ROS2 Node ---
class WebBridgeNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('web_bridge')
        self._loop = loop
        self._subs = []
        self._latest_control_states = {
            '/controls/expo_enabled': False,
            '/controls/precision_mode': False,
            '/controls/stabilize_enabled': False,
        }
        self._latest_control_states_lock = threading.Lock()

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
        controls_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subs.append(self.create_subscription(Twist, '/velocity_commands', self._on_velocity_commands, 10))
        self._subs.append(self.create_subscription(Float32MultiArray, '/arm_commands', self._on_arm_commands, 10))
        self._subs.append(self.create_subscription(Bool, '/controls/expo_enabled', self._on_expo_enabled, controls_qos))
        self._subs.append(self.create_subscription(Bool, '/controls/precision_mode', self._on_precision_mode, controls_qos))
        self._subs.append(self.create_subscription(Bool, '/controls/stabilize_enabled', self._on_stabilize_enabled, controls_qos))

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

        # GUI command publishers (browser -> ROS)
        self.expo_toggle_pub = self.create_publisher(Bool, '/gui_buttons/expo_enabled', 10)
        self.precision_mode_toggle_pub = self.create_publisher(Bool, '/gui_buttons/precision_mode', 10)
        self.stabilize_toggle_pub = self.create_publisher(Bool, '/gui_buttons/stabilize_enabled', 10)
        self.get_logger().info('WebBridge node started')

    def handle_ws_message(self, raw_payload: str):
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

        if topic == '/gui_buttons/expo_enabled':
            msg = Bool()
            msg.data = bool(data)
            self.expo_toggle_pub.publish(msg)
            return

        if topic == '/gui_buttons/precision_mode':
            msg = Bool()
            msg.data = bool(data)
            self.precision_mode_toggle_pub.publish(msg)
            return

        if topic == '/gui_buttons/stabilize_enabled':
            msg = Bool()
            msg.data = bool(data)
            self.stabilize_toggle_pub.publish(msg)
            return

        self.get_logger().debug(f'Ignoring unsupported websocket topic: {topic}')


    def _forward(self, topic: str, payload):
        """Thread-safe: store the latest value for periodic websocket flush."""
        with pending_lock:
            pending_messages[topic] = payload

    async def _flush_loop(self):
        while True:
            await asyncio.sleep(flush_interval_s)
            with pending_lock:
                if not pending_messages:
                    continue
                items = list(pending_messages.items())
                pending_messages.clear()

            for topic, payload in items:
                await broadcast({'topic': topic, 'data': payload})

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

    def _on_expo_enabled(self, msg: Bool):
        value = bool(msg.data)
        with self._latest_control_states_lock:
            self._latest_control_states['/controls/expo_enabled'] = value
        self._forward('/controls/expo_enabled', value)

    def _on_precision_mode(self, msg: Bool):
        value = bool(msg.data)
        with self._latest_control_states_lock:
            self._latest_control_states['/controls/precision_mode'] = value
        self._forward('/controls/precision_mode', value)

    def _on_stabilize_enabled(self, msg: Bool):
        value = bool(msg.data)
        with self._latest_control_states_lock:
            self._latest_control_states['/controls/stabilize_enabled'] = value
        self._forward('/controls/stabilize_enabled', value)

    def get_control_state_snapshot(self):
        with self._latest_control_states_lock:
            return dict(self._latest_control_states)

    def _on_video(self, topic: str, msg: CompressedImage):
        payload = {
            'format': msg.format,
            'data': list(msg.data),
        }
        self._forward(topic, payload)

# --- HTTP + WebSocket server ---
async def ws_handler(request):
    node = request.app['ros_node']
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    ws_clients.add(ws)

    # Send latest control states to avoid button-state desync after page refresh/reconnect.
    for topic, data in node.get_control_state_snapshot().items():
        await ws.send_str(json.dumps({'topic': topic, 'data': data}))

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
