// Update the on-screen joystick indicator and display neck values.
function setStick(horizontal, vertical) {
  const x = horizontal / 30;
  const y = -vertical / 30;
  stick.style.left = `${50 + x * 36}%`;
  stick.style.top = `${50 + y * 36}%`;
  $('neckReadout').textContent = `V ${vertical} | H ${horizontal}`;
}

// Immediately send a neck command to the robot over websocket.
function sendNeckNow(vertical, horizontal, duration = 0) {
  const payload = { type: 'neck', vertical, horizontal, duration };
  wsSend(payload);
  lastNeckSentAt = performance.now();
}


// Throttle neck commands so the robot receives smoother updates.
function sendNeck(vertical, horizontal, immediate = false, duration = 0) {
  setStick(horizontal, vertical);
  pendingNeck = { vertical, horizontal };
  const elapsed = performance.now() - lastNeckSentAt;
  if (immediate || elapsed > 25) {
    clearTimeout(neckTimer);
    sendNeckNow(vertical, horizontal, duration);
    return;
  }
  clearTimeout(neckTimer);
  neckTimer = setTimeout(() => {
    if (pendingNeck) sendNeckNow(pendingNeck.vertical, pendingNeck.horizontal, duration);
  }, 25 - elapsed);
}

// Convert a pointer event into neck angle values for the virtual joystick.
function joystickPoint(ev) {
  const rect = joystick.getBoundingClientRect();
  const p = ev.touches ? ev.touches[0] : ev;
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let x = (p.clientX - cx) / (rect.width / 2);
  let y = (p.clientY - cy) / (rect.height / 2);
  const len = Math.hypot(x, y);
  if (len > 1) { x /= len; y /= len; }
  return { horizontal: Math.round(x * 30), vertical: Math.round(-y * 30) };
}

// Begin joystick interaction when the user presses the control.
function startJoystick(ev) { joystickActive = true; moveJoystick(ev); }

// Send neck commands as the user moves the joystick control.
function moveJoystick(ev) {
  if (!joystickActive) return;
  ev.preventDefault();
  const p = joystickPoint(ev);
  sendNeck(p.vertical, p.horizontal);
}

// Stop joystick tracking when the user releases the control.
function endJoystick() { joystickActive = false; }

joystick.addEventListener('mousedown', startJoystick);
window.addEventListener('mousemove', moveJoystick);
window.addEventListener('mouseup', endJoystick);
joystick.addEventListener('touchstart', startJoystick, { passive: false });
window.addEventListener('touchmove', moveJoystick, { passive: false });
window.addEventListener('touchend', endJoystick);


let circleAnimationId = null;
let circleAngle = 0; // Текущий угол движения в радианах

function toggleCircle() {
  const btn = document.getElementById('circleToggle');

  // Если движение по кругу уже активно — останавливаем его
  if (circleAnimationId) {
    cancelAnimationFrame(circleAnimationId);
    circleAnimationId = null;
    btn.classList.remove('active');

    // Возвращаем джойстик и шею ровно в центр (0, 0)
    sendNeck(0, 0, true, 0.5);
    return;
  }

  const cosValue = Math.cos(circleAngle); // Горизонтальная ось (X)
  const sinValue = Math.sin(circleAngle); // Вертикальная ось (Y)
  // Переводим в максимальный рабочий диапазон вашего джойстика (-30 до 30)
  const horizontal = Math.round(cosValue * 30);
  const vertical = Math.round(sinValue * 30);
  sendNeck(vertical, horizontal, true, 0.5);
  // Запуск кругового движения
  btn.classList.add('active');

  function animate() {
    // Увеличиваем угол. 0.03 — скорость вращения (чем больше, тем быстрее)
    circleAngle += 0.03;

    // Вычисляем координаты на окружности от -1 до 1
    const cosValue = Math.cos(circleAngle); // Горизонтальная ось (X)
    const sinValue = Math.sin(circleAngle); // Вертикальная ось (Y)

    // Переводим в максимальный рабочий диапазон вашего джойстика (-30 до 30)
    const horizontal = Math.round(cosValue * 30);
    const vertical = Math.round(sinValue * 30);

    // Передаем координаты в вашу систему троттлинга шеи и отрисовки стика
    sendNeck(vertical, horizontal, true);

    // Запрашиваем следующий кадр анимации
    circleAnimationId = requestAnimationFrame(animate);
  }

  setTimeout(animate, 500);
}

// Привязываем событие клика к кнопке
document.getElementById('circleToggle').onclick = toggleCircle;

// ИНТЕГРАЦИЯ: Если пользователь сам схватил джойстик руками, автоматический круг должен выключиться
const originalStartJoystick = startJoystick;
startJoystick = function (ev) {
  if (circleAnimationId) {
    toggleCircle(); // Принудительно выключаем автоматический режим
  }
  originalStartJoystick(ev);
};
