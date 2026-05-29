const $ = (id) => document.getElementById(id);
const connection = $('connection');
const paint = $('paint');
const paintCtx = paint.getContext('2d');
const joystick = $('joystick');
const stick = $('stick');

let ws;
let audioWs;
let audioInWs;
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
let syncing = false;

let cameraStreamActive = false;
let cameraStreamInterval = null;
let localVideoEl = null;
let captureCanvas = null;
let captureCtx = null;
const STREAM_WIDTH = 640;
const STREAM_HEIGHT = 480;
const STREAM_FPS = 15;

let voiceStreamActive = false;
let micStream = null;
let voiceProcessor = null;
let voiceContext = null;

function applyPaintBackground(send = true) {
  const color = $('background').value;
  paintCtx.fillStyle = color;
  paintCtx.fillRect(0, 0, paint.width, paint.height);
  lastRemote = null;
  if (send) wsSend({type: 'clear', color});
}
applyPaintBackground(false);

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
connect();

function wsSend(data) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(data));
}

function renderStatus(s) {
  $('battery').textContent = s.battery ? `Батарея: ${s.battery.voltage}V ${s.battery.current}A` : 'Батарея: --';
  $('doa').textContent = s.doa === null ? 'DoA: --' : `DoA: ${s.doa} градусов`
}

function drawLocalLine(cmd) {
  paintCtx.strokeStyle = cmd.color;
  paintCtx.lineWidth = cmd.size * 2;
  paintCtx.lineCap = 'round';
  paintCtx.beginPath();
  paintCtx.moveTo(cmd.x0, cmd.y0);
  paintCtx.lineTo(cmd.x1, cmd.y1);
  paintCtx.stroke();
}

function applyRemoteClear(color) {
  if (color) $('background').value = color;
  applyPaintBackground(false);
}

function replayCanvasHistory(commands) {
  if (drawing || syncing) return;
  syncing = true;
  try {
    paintCtx.clearRect(0, 0, paint.width, paint.height);
    for (const cmd of commands) {
      if (cmd.type === 'clear') applyRemoteClear(cmd.color);
      if (cmd.type === 'draw') drawLocalLine(cmd);
    }
  } finally {
    syncing = false;
  }
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
  $('neckReadout').textContent = `V ${vertical} | H ${horizontal}`;
}

function sendNeckNow(vertical, horizontal) {
  const payload = {type: 'neck', vertical, horizontal, duration: 0.04};
  wsSend(payload);
  lastNeckSentAt = performance.now();
}

function sendNeck(vertical, horizontal, immediate = false) {
  setStick(horizontal, vertical);
  pendingNeck = {vertical, horizontal};
  const elapsed = performance.now() - lastNeckSentAt;
  if (immediate || elapsed > 25) {
    clearTimeout(neckTimer);
    sendNeckNow(vertical, horizontal);
    return;
  }
  clearTimeout(neckTimer);
  neckTimer = setTimeout(() => {
    if (pendingNeck) sendNeckNow(pendingNeck.vertical, pendingNeck.horizontal);
  }, 25 - elapsed);
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
function endJoystick() { joystickActive = false; }
joystick.addEventListener('mousedown', startJoystick);
window.addEventListener('mousemove', moveJoystick);
window.addEventListener('mouseup', endJoystick);
joystick.addEventListener('touchstart', startJoystick, {passive: false});
window.addEventListener('touchmove', moveJoystick, {passive: false});
window.addEventListener('touchend', endJoystick);
setStick(0, 0);

function sendEars() {
  debounceCommand(() => post('/api/ears', {left: +$('earL').value, right: +$('earR').value, duration: 0.0}), 50);
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
$('playMedia').onclick = () => post('/api/media', {video_path: $('videoPath').value, audio_path: $('audioPath').value, loop: $('mediaLoop').checked});
$('stopMedia').onclick = () => post('/api/media', {video_path: '__STOP__', audio_path: '__STOP__', loop: false});
$('clearPaint').onclick = () => applyPaintBackground(true);
$('background').onchange = () => applyPaintBackground(true);
$('listenAudio').onclick = toggleAudio;
$('localCameraBtn').onclick = toggleCameraStream;
$('voiceStreamBtn').onclick = toggleVoiceStream;

$('uploadMedia').onclick = async () => {
  const file = $('mediaUpload').files[0];
  if (!file) return alert('Ошибка загрузки файла');
  if (file.size > 100 * 1024 * 1024) return alert('Превышен максимальный размер файла');
  const progress = $('uploadProgress');
  progress.textContent = `Загрузка ${file.name}...`;
  $('uploadMedia').disabled = true;
  try {
    const form = new FormData();
    form.append('file', file);
    form.append('loop', $('mediaLoop').checked ? 'true' : 'false');
    const res = await fetch('/api/media/upload', {method: 'POST', body: form});
    const result = await res.json();
    progress.textContent = result.ok ? `${file.name} загружен` : `Ошибка: ${result.error || result.detail || 'unknown'}`;
    if (result.path) { $('videoPath').value = result.path; $('audioPath').value = result.path; }
  } catch (e) {
    progress.textContent = `Ошибка файла: ${e.message}`;
  } finally {
    $('uploadMedia').disabled = false;
    $('mediaUpload').value = '';
    setTimeout(() => { progress.textContent = ''; }, 3000);
  }
};

function point(ev) {
  const rect = paint.getBoundingClientRect();
  const p = ev.touches ? ev.touches[0] : ev;
  return {x: Math.round((p.clientX - rect.left) * paint.width / rect.width), y: Math.round((p.clientY - rect.top) * paint.height / rect.height)};
}
function sendDrawSegment(from, to, force = false) {
  const now = performance.now();
  if (!force && now - lastDrawSentAt < 45) return;
  wsSend({type: 'draw', x0: from.x, y0: from.y, x1: to.x, y1: to.y, color: $('paintColor').value, size: +$('brush').value, flush: force});
  lastDrawSentAt = now;
  lastRemote = {...to};
}
function startDraw(ev) {
  if (cameraStreamActive) return;
  ev.preventDefault();
  drawing = true;
  last = point(ev);
  lastRemote = {...last};
}
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
    return;
  }
  audioCtx = audioCtx || new (window.AudioContext || window.webkitAudioContext)();
  await audioCtx.resume();
  nextAudioTime = audioCtx.currentTime + 0.05;
  audioWs = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/audio_ws`);
  audioWs.binaryType = 'arraybuffer';
  audioWs.onopen = () => { $('listenAudio').classList.add('active'); $('listenAudio').textContent = 'Трансляция аудио'; };
  audioWs.onclose = () => { audioWs = null; $('listenAudio').classList.remove('active'); $('listenAudio').textContent = 'Отключено'; };
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

async function toggleCameraStream() {
  if (cameraStreamActive) return stopCameraStream();
  try {
    const stream = await navigator.mediaDevices.getUserMedia({video: {width: {ideal: STREAM_WIDTH}, height: {ideal: STREAM_HEIGHT}, frameRate: {ideal: STREAM_FPS}}});
    localVideoEl = document.createElement('video');
    localVideoEl.srcObject = stream;
    localVideoEl.autoplay = true;
    localVideoEl.muted = true;
    localVideoEl.playsInline = true;
    await localVideoEl.play();
    captureCanvas = document.createElement('canvas');
    captureCanvas.width = STREAM_WIDTH;
    captureCanvas.height = STREAM_HEIGHT;
    captureCtx = captureCanvas.getContext('2d');
    cameraStreamActive = true;
    $('localCameraBtn').textContent = 'Включить трансляцию своей камеры';
    $('localCameraBtn').classList.add('active');
    cameraStreamInterval = setInterval(sendCameraFrame, 1000 / STREAM_FPS);
  } catch (err) {
    alert('Ошибка трансляции камеры: ' + (err.message || err));
  }
}

function sendCameraFrame() {
  if (!cameraStreamActive || !ws || ws.readyState !== WebSocket.OPEN) return;
  captureCtx.drawImage(localVideoEl, 0, 0, STREAM_WIDTH, STREAM_HEIGHT);
  captureCanvas.toBlob(async (blob) => {
    if (!blob || !cameraStreamActive) return;
    const jpeg = new Uint8Array(await blob.arrayBuffer());
    const packet = new Uint8Array(12 + jpeg.length);
    packet[0] = 0x56; packet[1] = 0x49; packet[2] = 0x44; packet[3] = 0x00;
    writeUint32(packet, 4, STREAM_WIDTH);
    writeUint32(packet, 8, STREAM_HEIGHT);
    packet.set(jpeg, 12);
    ws.send(packet.buffer);
  }, 'image/jpeg', 0.72);
}

function writeUint32(buf, offset, value) {
  buf[offset] = value & 0xff;
  buf[offset + 1] = (value >> 8) & 0xff;
  buf[offset + 2] = (value >> 16) & 0xff;
  buf[offset + 3] = (value >> 24) & 0xff;
}

function stopCameraStream() {
  cameraStreamActive = false;
  clearInterval(cameraStreamInterval);
  cameraStreamInterval = null;
  if (localVideoEl?.srcObject) localVideoEl.srcObject.getTracks().forEach((track) => track.stop());
  localVideoEl = null;
  captureCanvas = null;
  captureCtx = null;
  $('localCameraBtn').textContent = 'Трансляция остановлена';
  $('localCameraBtn').classList.remove('active');
}

async function toggleVoiceStream() {
  if (voiceStreamActive) return stopVoiceStream();
  try {
    micStream = await navigator.mediaDevices.getUserMedia({audio: {echoCancellation: true, noiseSuppression: true, channelCount: 1}});
    voiceContext = voiceContext || new (window.AudioContext || window.webkitAudioContext)();
    await voiceContext.resume();
    audioInWs = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/audio_in_ws`);
    audioInWs.binaryType = 'arraybuffer';
    await new Promise((resolve, reject) => {
      audioInWs.onopen = resolve;
      audioInWs.onerror = reject;
    });
    const source = voiceContext.createMediaStreamSource(micStream);
    voiceProcessor = voiceContext.createScriptProcessor(1024, 1, 1);
    voiceProcessor.onaudioprocess = (e) => {
      if (!voiceStreamActive || !audioInWs || audioInWs.readyState !== WebSocket.OPEN) return;
      const pcm = resampleToInt16(e.inputBuffer.getChannelData(0), voiceContext.sampleRate, 16000);
      audioInWs.send(pcm.buffer);
    };
    source.connect(voiceProcessor);
    voiceProcessor.connect(voiceContext.destination);
    voiceStreamActive = true;
    $('voiceStreamBtn').textContent = 'Трансляция голоса';
    $('voiceStreamBtn').classList.add('active');
  } catch (err) {
    alert('Ошибка трансляции голоса: ' + (err.message || err));
    stopVoiceStream();
  }
}

function resampleToInt16(input, inputRate, targetRate) {
  const ratio = inputRate / targetRate;
  const outLen = Math.max(1, Math.floor(input.length / ratio));
  const out = new Int16Array(outLen);
  for (let i = 0; i < outLen; i++) {
    const src = i * ratio;
    const i0 = Math.floor(src);
    const i1 = Math.min(input.length - 1, i0 + 1);
    const frac = src - i0;
    const sample = input[i0] * (1 - frac) + input[i1] * frac;
    const s = Math.max(-1, Math.min(1, sample));
    out[i] = s < 0 ? s * 32768 : s * 32767;
  }
  return out;
}

function stopVoiceStream() {
  voiceStreamActive = false;
  if (voiceProcessor) voiceProcessor.disconnect();
  voiceProcessor = null;
  if (micStream) micStream.getTracks().forEach((track) => track.stop());
  micStream = null;
  if (audioInWs) audioInWs.close();
  audioInWs = null;
  $('voiceStreamBtn').textContent = 'Остановлена трансляция голоса';
  $('voiceStreamBtn').classList.remove('active');
}
