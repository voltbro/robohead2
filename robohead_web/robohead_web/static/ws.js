// Create and manage the main websocket connection for robot status and canvas updates.
function connect() {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) ws.close();
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => connection.textContent = 'Подключено';
  ws.onclose = () => { connection.textContent = 'Отключено'; setTimeout(connect, 700); };
  ws.onerror = () => connection.textContent = 'Ошибка подключения';
  ws.onmessage = (ev) => {
    if (typeof ev.data !== 'string') return;
    const msg = JSON.parse(ev.data);
    if (msg.type === 'status') renderStatus(msg.data);
    if (msg.type === 'canvas_history') replayCanvasHistory(msg.commands || []);
    if (msg.type === 'draw') drawLocalLine(msg);
    if (msg.type === 'clear') applyRemoteClear(msg.color);
  };
}

// Send a JSON command to the robot websocket if the connection is open.
function wsSend(data) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(data));
}

// Display robot status updates in the UI.
function renderStatus(s) {
  $('battery').textContent = s.battery ? `Батарея: ${s.battery.voltage}V ${s.battery.current}A` : 'Батарея: --';
  $('doa').textContent = s.doa === null ? 'DoA: --' : `DoA: ${s.doa} градусов`;
}
