const $ = (id) => document.getElementById(id);
const connection = $('connection');
const paint = $('paint');
const paintCtx = paint.getContext('2d');
const joystick = $('joystick');
const stick = $('stick');
let ws;
let audioWs;
let audioCtx;
let nextAudioTime = 0;
let audioSampleRate = 16000;
let drawing = false;
let last = null;
let lastRemote = null;
let lastDrawSentAt = 0;
let commandTimer = null;
let joystickActive = false;
let lastNeckSentAt = 0;
let neckTimer = null;
let pendingNeck = null;

function applyPaintBackground(send = true) {
  const color = $('background').value;
  paintCtx.fillStyle = color;
  paintCtx.fillRect(0, 0, paint.width, paint.height);
  lastRemote = null;
  if (send) ws?.send(JSON.stringify({type: 'clear', color}));
}
applyPaintBackground(false);

function connect() {
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => connection.textContent = 'connected';
  ws.onclose = () => { connection.textContent = 'disconnected'; setTimeout(connect, 1000); };
  ws.onerror = () => connection.textContent = 'error';
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'status') renderStatus(msg.data);
  };
}
connect();

function renderStatus(s) {
  $('battery').textContent = s.battery ? `battery: ${s.battery.voltage}V ${s.battery.current}A` : 'battery: --';
  $('doa').textContent = s.doa === null ? 'doa: --' : `doa: ${s.doa} deg`;
}

async function post(url, data) {
  const res = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data)});
  return res.json().catch(() => ({}));
}

function debounceCommand(fn, delay = 70) {
  clearTimeout(commandTimer);
  commandTimer = setTimeout(fn, delay);
}

function setStick(horizontal, vertical) {
  const x = horizontal / 30;
  const y = -vertical / 30;
  stick.style.left = `${50 + x * 36}%`;
  stick.style.top = `${50 + y * 36}%`;
  $('neckReadout').textContent = `V ${vertical} / H ${horizontal}`;
}

function sendNeckNow(vertical, horizontal) {
  const payload = {type: 'neck', vertical, horizontal, duration: 0.06};
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(payload));
  else post('/api/neck', payload);
  lastNeckSentAt = performance.now();
}

function sendNeck(vertical, horizontal, immediate = false) {
  setStick(horizontal, vertical);
  pendingNeck = {vertical, horizontal};
  const elapsed = performance.now() - lastNeckSentAt;
  if (immediate || elapsed > 45) {
    clearTimeout(neckTimer);
    sendNeckNow(vertical, horizontal);
    return;
  }
  clearTimeout(neckTimer);
  neckTimer = setTimeout(() => {
    if (pendingNeck) sendNeckNow(pendingNeck.vertical, pendingNeck.horizontal);
  }, 45 - elapsed);
}

function joystickPoint(ev) {
  const rect = joystick.getBoundingClientRect();
  const p = ev.touches ? ev.touches[0] : ev;
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let x = (p.clientX - cx) / (rect.width / 2);
  let y = (p.clientY - cy) / (rect.height / 2);
  const len = Math.hypot(x, y);
  if (len > 1) { x /= len; y /= len; }
  return {horizontal: Math.round(x * 30), vertical: Math.round(-y * 30)};
}

function startJoystick(ev) { joystickActive = true; moveJoystick(ev); }
function moveJoystick(ev) {
  if (!joystickActive) return;
  ev.preventDefault();
  const p = joystickPoint(ev);
  sendNeck(p.vertical, p.horizontal);
}
function endJoystick() { if (!joystickActive) return; joystickActive = false; sendNeck(0, 0, true); }
joystick.addEventListener('mousedown', startJoystick);
window.addEventListener('mousemove', moveJoystick);
window.addEventListener('mouseup', endJoystick);
joystick.addEventListener('touchstart', startJoystick, {passive: false});
window.addEventListener('touchmove', moveJoystick, {passive: false});
window.addEventListener('touchend', endJoystick);
setStick(0, 0);

function sendEars() {
  post('/api/ears', {left: +$('earL').value, right: +$('earR').value, duration: 0.1});
}
$('earL').oninput = sendEars;
$('earR').oninput = sendEars;
document.querySelectorAll('[data-ears]').forEach((b) => b.onclick = () => {
  const [l, r] = b.dataset.ears.split(',').map(Number);
  $('earL').value = l;
  $('earR').value = r;
  sendEars();
});

$('speak').onclick = () => post('/api/tts', {text: $('ttsText').value, voice: $('voice').value, play: true});
$('volume').onchange = () => post('/api/volume', {value: +$('volume').value});
$('setLed').onclick = () => post('/api/led/color', {color: $('ledColor').value});
$('brightness').onchange = () => post('/api/led/brightness', {value: +$('brightness').value});
document.querySelectorAll('[data-mode]').forEach((b) => b.onclick = () => post('/api/led/mode', {mode: +b.dataset.mode}));
$('playMedia').onclick = () => post('/api/media', {video_path: $('videoPath').value, audio_path: $('audioPath').value, loop: false});
$('stopMedia').onclick = () => post('/api/media', {video_path: '__STOP__', audio_path: '__STOP__', loop: false});
$('clearPaint').onclick = () => applyPaintBackground(true);
$('background').onchange = () => applyPaintBackground(true);
$('listenAudio').onclick = toggleAudio;

function point(ev) {
  const rect = paint.getBoundingClientRect();
  const p = ev.touches ? ev.touches[0] : ev;
  return {x: Math.round((p.clientX - rect.left) * paint.width / rect.width), y: Math.round((p.clientY - rect.top) * paint.height / rect.height)};
}
function sendDrawSegment(from, to, force = false) {
  const now = performance.now();
  if (!force && now - lastDrawSentAt < 45) return;
  ws?.send(JSON.stringify({type: 'draw', x0: from.x, y0: from.y, x1: to.x, y1: to.y, color: $('paintColor').value, size: +$('brush').value}));
  lastDrawSentAt = now;
  lastRemote = {...to};
}
function startDraw(ev) { ev.preventDefault(); drawing = true; last = point(ev); lastRemote = {...last}; }
function moveDraw(ev) {
  if (!drawing) return;
  ev.preventDefault();
  const p = point(ev);
  const color = $('paintColor').value;
  const size = +$('brush').value;
  paintCtx.strokeStyle = color;
  paintCtx.lineWidth = size * 2;
  paintCtx.lineCap = 'round';
  paintCtx.beginPath();
  paintCtx.moveTo(last.x, last.y);
  paintCtx.lineTo(p.x, p.y);
  paintCtx.stroke();
  sendDrawSegment(lastRemote || last, p, false);
  last = p;
}
function endDraw() {
  if (drawing && lastRemote && last) sendDrawSegment(lastRemote, last, true);
  drawing = false;
  last = null;
  lastRemote = null;
}
paint.addEventListener('mousedown', startDraw);
paint.addEventListener('mousemove', moveDraw);
window.addEventListener('mouseup', endDraw);
paint.addEventListener('touchstart', startDraw, {passive: false});
paint.addEventListener('touchmove', moveDraw, {passive: false});
window.addEventListener('touchend', endDraw);

async function toggleAudio() {
  if (audioWs) {
    audioWs.close();
    audioWs = null;
    $('listenAudio').classList.remove('active');
    $('listenAudio').textContent = 'Listen';
    return;
  }
  audioCtx = audioCtx || new (window.AudioContext || window.webkitAudioContext)();
  await audioCtx.resume();
  nextAudioTime = audioCtx.currentTime + 0.05;
  audioWs = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/audio_ws`);
  audioWs.binaryType = 'arraybuffer';
  audioWs.onopen = () => { $('listenAudio').classList.add('active'); $('listenAudio').textContent = 'Listening'; };
  audioWs.onclose = () => { audioWs = null; $('listenAudio').classList.remove('active'); $('listenAudio').textContent = 'Listen'; };
  audioWs.onmessage = (ev) => {
    if (typeof ev.data === 'string') {
      const cfg = JSON.parse(ev.data);
      if (cfg.sampleRate) audioSampleRate = cfg.sampleRate;
      return;
    }
    playPcm(ev.data);
  };
}

function playPcm(buffer) {
  if (!audioCtx) return;
  const input = new Int16Array(buffer);
  if (!input.length) return;
  const ratio = audioCtx.sampleRate / audioSampleRate;
  const outLen = Math.max(1, Math.round(input.length * ratio));
  const audioBuffer = audioCtx.createBuffer(1, outLen, audioCtx.sampleRate);
  const out = audioBuffer.getChannelData(0);
  for (let i = 0; i < outLen; i++) {
    const src = i / ratio;
    const i0 = Math.floor(src);
    const i1 = Math.min(input.length - 1, i0 + 1);
    const frac = src - i0;
    out[i] = ((input[i0] * (1 - frac) + input[i1] * frac) / 32768) * 1.5;
  }
  const source = audioCtx.createBufferSource();
  source.buffer = audioBuffer;
  source.connect(audioCtx.destination);
  if (nextAudioTime < audioCtx.currentTime || nextAudioTime - audioCtx.currentTime > 0.3) nextAudioTime = audioCtx.currentTime + 0.04;
  source.start(nextAudioTime);
  nextAudioTime += audioBuffer.duration;
}
