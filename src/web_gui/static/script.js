const ws = new WebSocket(`ws://${location.host}/ws`);

ws.onclose = () => setTimeout(() => location.reload(), 2000);