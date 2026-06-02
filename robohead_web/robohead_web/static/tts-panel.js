// Send text-to-speech requests to the robot when the speak button is clicked.
$('speak').onclick = () => post('/api/tts', {text: $('ttsText').value, voice: $('voice').value, play: true});
// Send volume changes to the robot when the slider is adjusted.
$('volume').onchange = () => post('/api/volume', {value: +$('volume').value});
