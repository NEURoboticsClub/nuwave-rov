const ws = new WebSocket(`ws://${location.host}/ws`);

window.ws = ws;

ws.onclose = () => setTimeout(() => location.reload(), 2000);