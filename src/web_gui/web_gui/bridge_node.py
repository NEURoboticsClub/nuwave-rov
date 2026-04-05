import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import FluidPressure, Temperature, Image
from ament_index_python.packages import get_package_share_directory
import asyncio
import json
import threading
import aiohttp
import os
from aiohttp import web

# --- Shared state: connected WebSocket clients ---
ws_clients: set[web.WebSocketResponse] = set()

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

        # Thruster telemetry: /thruster/thruster_0..7
        for thruster_id in range(8):
            topic = f'/thruster/thruster_{thruster_id}'
            self._subs.append(
                self.create_subscription(
                    Float32,
                    topic,
                    lambda msg, topic=topic: self._on_scalar(topic, msg),
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

        # Arm motor telemetry: /arm/arm_motor_0..5
        for arm_motor_id in range(6):
            topic = f'/arm/arm_motor_{arm_motor_id}'
            self._subs.append(
                self.create_subscription(
                    Float32,
                    topic,
                    lambda msg, topic=topic: self._on_scalar(topic, msg),
                    10,
                )
            )

        # Depth telemetry
        self._subs.append(self.create_subscription(Float32, '/depth/depth', self._on_depth, 10))
        self._subs.append(self.create_subscription(FluidPressure, '/depth/pressure', self._on_pressure, 10))
        self._subs.append(self.create_subscription(Temperature, '/depth/temperature', self._on_temperature, 10))
        self._subs.append(self.create_subscription(Float32MultiArray, '/depth/depth_array', self._on_depth_array, 10))

        # Command topics
        self._subs.append(self.create_subscription(Twist, '/velocity_commands', self._on_velocity_commands, 10))
        self._subs.append(self.create_subscription(Float32MultiArray, '/arm_commands', self._on_arm_commands, 10))

        # Camera topics
        for camera_id in range(4):
            topic = f'/camera_{camera_id}/image/compressed'
            self._subs.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, topic=topic: self._on_video(topic, msg),
                    10,
                )
            )
        self.get_logger().info('WebBridge node started')

    def _forward(self, topic: str, payload):
        """Thread-safe: schedule broadcast onto the asyncio loop."""
        data = {'topic': topic, 'data': payload}
        asyncio.run_coroutine_threadsafe(broadcast(data), self._loop)

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

    def _on_video(self, topic: str, msg: Image):
        payload = {
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'step': msg.step,
            'data': list(msg.data),
        }
        self._forward(topic, payload)

# --- HTTP + WebSocket server ---
async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    ws_clients.add(ws)
    try:
        async for _ in ws:  # keep connection alive
            pass
    finally:
        ws_clients.discard(ws)
    return ws

def build_app():
    app = web.Application()
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
    app = build_app()
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, 'localhost', 8080)
    loop.run_until_complete(site.start())
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
