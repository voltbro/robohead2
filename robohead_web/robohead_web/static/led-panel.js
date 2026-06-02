// Send LED color and brightness changes to the robot when controls are used.
$('setLed').onclick = () => post('/api/led/color', {color: $('ledColor').value});
// Send LED brightness adjustments to the robot when the slider changes.
$('brightness').onchange = () => post('/api/led/brightness', {value: +$('brightness').value});
// Send LED mode selection commands when mode buttons are clicked.
document.querySelectorAll('[data-mode]').forEach((b) => b.onclick = () => post('/api/led/mode', {mode: +b.dataset.mode}));
