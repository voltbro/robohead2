// Send ear position values to the robot using a debounced service call.
function sendEars() {
  debounceCommand(() => post('/api/ears', {left: +$('earL').value, right: +$('earR').value, duration: 0.0}), 5);
}

$('earL').oninput = sendEars;
$('earR').oninput = sendEars;
// Attach preset ear angle buttons so each click sets the ear positions.
document.querySelectorAll('[data-ears]').forEach((b) => b.onclick = () => {
  const [l, r] = b.dataset.ears.split(',').map(Number);
  $('earL').value = l;
  $('earR').value = r;
  sendEars();
});


let waveAnimationId = null;
let waveAngle = 0; // Текущая фаза анимации в радианах

function toggleWave() {
  const btn = $('waveToggle');
  
  // Если волна уже запущена — останавливаем её
  if (waveAnimationId) {
    cancelAnimationFrame(waveAnimationId);
    waveAnimationId = null;
    btn.classList.remove('active'); // Можно стилизовать в CSS
    return;
  }

  // Запуск анимации
  btn.classList.add('active');
  
  function animate() {
    // Увеличиваем угол. Скорость регулируется шагом (0.05 — плавно)
    waveAngle += 0.05; 

    // Вычисляем значение синуса (от -1 до 1)
    const sinValue = Math.sin(waveAngle); 

    // Переводим в диапазон углов от -90 до 90
    const angleL = Math.round(sinValue * 90);
    // Для противофазы инвертируем знак (умножаем на -1)
    const angleR = Math.round(-sinValue * 90); 

    // Записываем значения в ползунки
    $('earL').value = angleL;
    $('earR').value = angleR;

    // Отправляем текущее состояние по сети
    sendEars();

    // Запрашиваем следующий кадр анимации (60 кадров в секунду)
    waveAnimationId = requestAnimationFrame(animate);
  }

  animate();
}

// Привязываем событие клика к кнопке волны
$('waveToggle').onclick = toggleWave;

// Дополнительно: останавливать волну, если пользователь нажал кнопку "Центр"
document.querySelectorAll('[data-ears]').forEach((b) => {
  const originalClick = b.onclick;
  b.onclick = () => {
    if (waveAnimationId) toggleWave(); // Выключаем волну, если она активна
    originalClick();
  };
});
