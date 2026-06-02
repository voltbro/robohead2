// Enable or disable webcam streaming from the browser to the robot.
async function toggleCameraStream() {
  if (cameraStreamActive) return stopCameraStream();

  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {width: {ideal: STREAM_WIDTH}, height: {ideal: STREAM_HEIGHT}, frameRate: {ideal: STREAM_FPS}},
    });
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
    $('localCameraBtn').classList.add('active');
    cameraStreamInterval = setInterval(sendCameraFrame, 1000 / STREAM_FPS);
  } catch (err) {
    alert('Ошибка трансляции камеры: ' + (err.message || err));
  }
}

// Capture a video frame, package it as a video packet, and send it over websocket.
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

// Stop the local camera feed and clean up browser video resources.
function stopCameraStream() {
  cameraStreamActive = false;
  clearInterval(cameraStreamInterval);
  cameraStreamInterval = null;
  if (localVideoEl?.srcObject) localVideoEl.srcObject.getTracks().forEach((track) => track.stop());
  localVideoEl = null;
  captureCanvas = null;
  captureCtx = null;
  $('localCameraBtn').classList.remove('active');
}

// Attach the local camera button to start and stop webcam streaming.
$('localCameraBtn').onclick = toggleCameraStream;
