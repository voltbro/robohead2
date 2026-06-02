// Start or stop streaming captured microphone audio to the robot.
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
    $('voiceStreamBtn').classList.add('active');
  } catch (err) {
    alert('Ошибка трансляции голоса: ' + (err.message || err));
    stopVoiceStream();
  }
}

// Resample floating point audio samples to 16-bit PCM at the target rate.
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

// Stop microphone capture and close the voice websocket connection.
function stopVoiceStream() {
  voiceStreamActive = false;
  if (voiceProcessor) voiceProcessor.disconnect();
  voiceProcessor = null;
  if (micStream) micStream.getTracks().forEach((track) => track.stop());
  micStream = null;
  if (audioInWs) audioInWs.close();
  audioInWs = null;
  $('voiceStreamBtn').classList.remove('active');
}

// Attach the voice stream button to start or stop microphone streaming.
$('voiceStreamBtn').onclick = toggleVoiceStream;
