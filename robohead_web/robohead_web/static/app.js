const $ = (id) => document.getElementById(id);
const connection = $('connection');
const paint = $('paint');
const paintCtx = paint.getContext('2d');
const joystick = $('joystick');
const stick = $('stick');
const mediaUpload = $('mediaUpload');
const uploadMediaBtn = $('uploadMedia');
const mediaLoop = $('mediaLoop');
const uploadProgress = $('uploadProgress');

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
let isSyncing = false;
let pendingSync = null;  // Очередь для отложенной синхронизации

function applyPaintBackground(send = true) {
  const color = $('background').value;
  paintCtx.fillStyle = color;
  paintCtx.fillRect(0, 0, paint.width, paint.height);
  lastRemote = null;
  if (send) ws?.send(JSON.stringify({type: 'clear', color}));
}
applyPaintBackground(false);


uploadMediaBtn.onclick = async () => {
  const file = mediaUpload.files[0];
  if (!file) {
    alert('Выберите файл для загрузки');
    return;
  }
  
  // Валидация размера (макс 100 МБ)
  if (file.size > 100 * 1024 * 1024) {
    alert('Файл слишком большой (макс 100 МБ)');
    return;
  }
  
  const loop = mediaLoop.checked;
  uploadProgress.textContent = `Загрузка ${file.name}...`;
  uploadMediaBtn.disabled = true;
  
  try {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('loop', loop ? 'true' : 'false');
    
    const res = await fetch('/api/media/upload', {
      method: 'POST',
      body: formData
    });
    
    const result = await res.json();
    
    if (result.ok) {
      uploadProgress.textContent = `✅ ${file.name} загружен и воспроизводится`;
      // Обновляем поля путей для отображения
      $('videoPath').value = result.path;
      $('audioPath').value = result.path; // Для видео со звуком — одинаковый путь
    } else {
      uploadProgress.textContent = `❌ Ошибка: ${result.error}`;
    }
  } catch (e) {
    uploadProgress.textContent = `❌ Ошибка сети: ${e.message}`;
    console.error('Upload error:', e);
  } finally {
    uploadMediaBtn.disabled = false;
    // Очищаем инпут, чтобы можно было загрузить тот же файл повторно
    mediaUpload.value = '';
    setTimeout(() => { uploadProgress.textContent = ''; }, 3000);
  }
};

function connect() {
  if (ws) {
    ws.onopen = null;
    ws.onclose = null;
    ws.onerror = null;
    ws.onmessage = null;
    if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
      ws.close();  // принудительное закрытие
    }
  }

  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => connection.textContent = 'Подключено';
  ws.onclose = () => { connection.textContent = 'Отключено'; setTimeout(connect, 500); };
  ws.onerror = () => connection.textContent = 'Ошибка';
  ws.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data);

      if (msg.type === 'status') {
        renderStatus(msg.data);
      } 
      
      else if (msg.type === 'canvas_history') {
        // При подключении получаем полную историю
        replayCanvasHistory(msg.commands);
      } 
      
      else if (msg.type === 'draw') {
        // 🟢 Пришло рисование от ДРУГОГО пользователя
        // Рисуем линию локально, но НЕ отправляем её обратно на сервер
        paintCtx.strokeStyle = msg.color;
        paintCtx.lineWidth = msg.size * 2;
        paintCtx.lineCap = 'round';
        paintCtx.beginPath();
        paintCtx.moveTo(msg.x0, msg.y0);
        paintCtx.lineTo(msg.x1, msg.y1);
        paintCtx.stroke();
        
        // Обновляем lastRemote, чтобы если мы начнем рисовать, линия продолжилась верно
        lastRemote = {x: msg.x1, y: msg.y1};
      } 
      
      else if (msg.type === 'clear') {
        // 🟢 Пришла очистка от ДРУГОГО пользователя
        applyPaintBackground(false); // false = не отправлять ответ
        $('background').value = msg.color;
      }

    } catch (e) {
      console.error('WS parse error:', e);
    }
  };
}
connect();

function renderStatus(s) {
  $('battery').textContent = s.battery ? `Батарея: ${s.battery.voltage}V ${s.battery.current}A` : 'Батарея: --';
  $('doa').textContent = s.doa ? `DoA: ${s.doa} градусов` : 'DoA: --';
}
function replayCanvasHistory(commands) {
  if (isSyncing) return;  // Защита от рекурсии
  isSyncing = true;
  
  try {
    // Сохраняем текущее состояние, если что-то рисуем
    let tempImageData = null;
    if (drawing && last) {
      tempImageData = paintCtx.getImageData(0, 0, paint.width, paint.height);
    }
    
    // Очищаем и перерисовываем историю
    paintCtx.clearRect(0, 0, paint.width, paint.height);
    
    for (const cmd of commands) {
      if (cmd.type === 'clear') {
        paintCtx.fillStyle = cmd.color;
        paintCtx.fillRect(0, 0, paint.width, paint.height);
        $('background').value = cmd.color;
      } else if (cmd.type === 'draw') {
        paintCtx.strokeStyle = cmd.color;
        paintCtx.lineWidth = cmd.size * 2;
        paintCtx.lineCap = 'round';
        paintCtx.beginPath();
        paintCtx.moveTo(cmd.x0, cmd.y0);
        paintCtx.lineTo(cmd.x1, cmd.y1);
        paintCtx.stroke();
      }
    }
    
    // Восстанавливаем текущую линию, если она была
    if (tempImageData) {
      paintCtx.putImageData(tempImageData, 0, 0);
    }
    
    // Сбрасываем lastRemote, но НЕ drawing!
    lastRemote = null;
    
  } finally {
    isSyncing = false;
    
    // 🟢 Применяем отложенную синхронизацию, если она пришла
    if (pendingSync && !drawing) {
      const sync = pendingSync;
      pendingSync = null;
      replayCanvasHistory(sync);
    }
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
  const payload = {type: 'neck', vertical, horizontal, duration: 0.00};
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(payload));
  else post('/api/neck', payload);
  lastNeckSentAt = performance.now();
}

function sendNeck(vertical, horizontal, immediate = false) {
  setStick(horizontal, vertical);
  pendingNeck = {vertical, horizontal};
  const elapsed = performance.now() - lastNeckSentAt;
  if (immediate || elapsed > 10) {
    clearTimeout(neckTimer);
    sendNeckNow(vertical, horizontal);
    return;
  }
  clearTimeout(neckTimer);
  neckTimer = setTimeout(() => {
    if (pendingNeck) sendNeckNow(pendingNeck.vertical, pendingNeck.horizontal);
  }, 10 - elapsed);
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
  post('/api/ears', {left: +$('earL').value, right: +$('earR').value, duration: 0.0});
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
// $('playMedia').onclick = () => post('/api/media', {video_path: $('videoPath').value, audio_path: $('audioPath').value, loop: false});

$('playMedia').onclick = () => {
  const videoPath = $('videoPath').value;
  const audioPath = $('audioPath').value;
  const loop = mediaLoop.checked; // Берём значение из чекбокса
  
  post('/api/media', {
    video_path: videoPath,
    audio_path: audioPath,
    loop: loop
  });
};

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
  if (drawing && lastRemote && last) {
    sendDrawSegment(lastRemote, last, true);
  }
  drawing = false;
  last = null;
  lastRemote = null;
  
  // 🟢 Если во время рисования пришла синхронизация — применим её сейчас
  if (pendingSync) {
    const sync = pendingSync;
    pendingSync = null;
    replayCanvasHistory(sync);
  }
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
