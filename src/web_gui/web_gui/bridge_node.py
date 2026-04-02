import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, String  # swap for your message types
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

        # Subscribe to topics topics you need
        self.create_subscription(Float64, '/imu/depth', self._on_depth, 10)
        self.create_subscription(String, '/status', self._on_status, 10)
        self.get_logger().info('WebBridge node started')

    def _forward(self, topic: str, payload):
        """Thread-safe: schedule broadcast onto the asyncio loop."""
        data = {'topic': topic, 'data': payload}
        asyncio.run_coroutine_threadsafe(broadcast(data), self._loop)

    def _on_depth(self, msg: Float64):
        self._forward('/imu/depth', msg.data)

    def _on_status(self, msg: String):
        self._forward('/status', msg.data)

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
