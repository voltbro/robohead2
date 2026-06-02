// Toggle browser audio streaming from the robot.
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
  audioWs.onopen = () => { $('listenAudio').classList.add('active'); };
  audioWs.onclose = () => { audioWs = null; $('listenAudio').classList.remove('active'); };
  audioWs.onmessage = (ev) => {
    if (typeof ev.data === 'string') {
      const cfg = JSON.parse(ev.data);
      if (cfg.sampleRate) audioSampleRate = cfg.sampleRate;
      return;
    }
    playPcm(ev.data);
  };
}

// Decode raw PCM audio and play it through the browser audio context.
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

// Attach the audio toggle button to start and stop streaming.
$('listenAudio').onclick = toggleAudio;
