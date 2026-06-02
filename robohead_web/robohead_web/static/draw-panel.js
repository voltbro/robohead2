// Fill the paint canvas with the current background color and optionally broadcast the clear action.
function applyPaintBackground(send = true) {
  const color = $('backgroundColor').value;
  paintCtx.fillStyle = color;
  paintCtx.fillRect(0, 0, paint.width, paint.height);
  lastRemote = null;
  if (send) wsSend({type: 'clear', color});
}

// Draw a remote or local line segment on the paint canvas.
function drawLocalLine(cmd) {
  paintCtx.strokeStyle = cmd.color;
  paintCtx.lineWidth = cmd.size * 2;
  paintCtx.lineCap = 'round';
  paintCtx.beginPath();
  paintCtx.moveTo(cmd.x0, cmd.y0);
  const x1 = (cmd.x0 === cmd.x1 && cmd.y0 === cmd.y1) ? cmd.x1 + 0.1 : cmd.x1;
  paintCtx.lineTo(x1, cmd.y1);
  paintCtx.stroke();
}

// Apply a remote clear command by resetting the background color locally.
function applyRemoteClear(color) {
  if (color) $('backgroundColor').value = color;
  applyPaintBackground(false);
}

// Replay all stored canvas commands to rebuild the shared drawing state.
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

// Convert a mouse or touch event into canvas coordinates.
function point(ev) {
  const rect = paint.getBoundingClientRect();
  const p = ev.touches ? ev.touches[0] : ev;
  return {
    x: Math.round((p.clientX - rect.left) * paint.width / rect.width),
    y: Math.round((p.clientY - rect.top) * paint.height / rect.height),
  };
}

// Send a single drawing segment to the websocket as a draw command.
function sendDrawSegment(from, to, force = false) {
  const now = performance.now();
  if (!force && now - lastDrawSentAt < 10) return;
  wsSend({
    type: 'draw',
    x0: from.x,
    y0: from.y,
    x1: to.x,
    y1: to.y,
    color: $('brushColor').value,
    size: +$('brushThickness').value,
    flush: force,
  });
  lastDrawSentAt = now;
  lastRemote = {...to};
}

// Begin a local drawing gesture when the user presses on the canvas.
function startDraw(ev) {
  if (cameraStreamActive) return;
  ev.preventDefault();
  drawing = true;
  last = point(ev);
  lastRemote = {...last};

  paintCtx.strokeStyle = $('brushColor').value;
  paintCtx.lineWidth = +$('brushThickness').value * 2;
  paintCtx.lineCap = 'round';
  paintCtx.beginPath();
  paintCtx.moveTo(last.x, last.y);
  // paintCtx.lineTo(last.x, last.y); // Рисуем линию в саму себя (получится точка)
  paintCtx.lineTo(last.x + 0.1, last.y); // +0.1 - хак для IOS Safari
  paintCtx.stroke();
}

// Continue drawing as the user moves the pointer and send the segment to peers.
function moveDraw(ev) {
  if (!drawing) return;
  ev.preventDefault();
  const p = point(ev);
  paintCtx.strokeStyle = $('brushColor').value;
  paintCtx.lineWidth = +$('brushThickness').value * 2;
  paintCtx.lineCap = 'round';
  paintCtx.beginPath();
  paintCtx.moveTo(last.x, last.y);
  paintCtx.lineTo(p.x, p.y);
  paintCtx.stroke();
  sendDrawSegment(lastRemote || last, p, false);
  last = p;
}

// Finish the current drawing gesture and flush the final segment.
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
