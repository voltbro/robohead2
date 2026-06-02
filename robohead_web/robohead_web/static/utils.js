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

// Send a JSON POST request and return the parsed response.
async function post(url, data) {
  const res = await fetch(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(data),
  });
  return res.json().catch(() => ({}));
}

// Delay a command so rapid UI changes do not flood the server.
function debounceCommand(fn, delay = 70) {
  clearTimeout(commandTimer);
  commandTimer = setTimeout(fn, delay);
}

// Write a 32-bit unsigned integer into a byte buffer in little-endian order.
function writeUint32(buf, offset, value) {
  buf[offset] = value & 0xff;
  buf[offset + 1] = (value >> 8) & 0xff;
  buf[offset + 2] = (value >> 16) & 0xff;
  buf[offset + 3] = (value >> 24) & 0xff;
}
