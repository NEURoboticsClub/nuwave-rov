const ws = new WebSocket(`ws://${location.host}/ws`);

ws.onmessage = (event) => {
    const { topic, data } = JSON.parse(event.data);

    if (topic === '/imu/depth')
    document.getElementById('depth').textContent = `Depth: ${data.toFixed(2)} m`;
    if (topic === '/status')
    document.getElementById('status').textContent = `Status: ${data}`;
};

ws.onclose = () => setTimeout(() => location.reload(), 2000);