// Play selected media on the robot when the play button is pressed.
$('playMedia').onclick = () => post('/api/media', {video_path: $('videoPath').value, audio_path: $('audioPath').value, loop: $('mediaLoop').checked});
// Stop media playback when the stop button is pressed.
$('stopMedia').onclick = () => post('/api/media', {video_path: '__STOP__', audio_path: '__STOP__', loop: false});
// Clear the painting canvas and notify other clients.
$('clearPaint').onclick = () => applyPaintBackground(true);
// Broadcast background color changes when the user updates the color picker.
$('backgroundColor').onchange = () => applyPaintBackground(true);

// Upload media from the browser to the robot and update the UI with progress.
// Upload a media file from the browser to the robot and update the UI progress.
// Upload media from the browser to the robot and update the UI with progress.
$('uploadMedia').onclick = async () => {
  const file = $('mediaUpload').files[0];
  if (!file) return alert('Выберите файл для загрузки');
  if (file.size > 100 * 1024 * 1024) return alert('Превышен максимальный размер файла (100 МБ)');

  const progress = $('uploadProgress');
  progress.textContent = `Загрузка ${file.name}...`;
  $('uploadMedia').disabled = true;

  try {
    const form = new FormData();
    form.append('file', file);
    form.append('loop', $('mediaLoop').checked ? 'true' : 'false');

    const res = await fetch('/api/media/upload', { method: 'POST', body: form });
    
    // ИСПРАВЛЕНО: Проверяем успешность HTTP-запроса через res.ok, а не через json
    if (!res.ok) {
      const errResult = await res.json().catch(() => ({}));
      throw new Error(errResult.error || errResult.detail || `HTTP ${res.status}`);
    }

    const result = await res.json();
    progress.textContent = `${file.name} успешно загружен`;

    // Автоматически подставляем путь к загруженному файлу в поля путей
    if (result.path) {
      if ($('videoPath')) $('videoPath').value = result.path;
      if ($('audioPath')) $('audioPath').value = result.path;
    } else if (result.video_path || result.audio_path) { 
      // Дополнительная проверка, если сервер возвращает раздельные пути
      if ($('videoPath')) $('videoPath').value = result.video_path || '';
      if ($('audioPath')) $('audioPath').value = result.audio_path || '';
    }

  } catch (e) {
    progress.textContent = `Ошибка загрузки: ${e.message}`;
    console.error(e);
  } finally {
    $('uploadMedia').disabled = false;
    
    // ИСПРАВЛЕНО: Очищаем поле выбора файла с небольшой задержкой для стабильности на мобильных
    setTimeout(() => { 
      $('mediaUpload').value = ''; 
    }, 100);

    // Стираем статусное сообщение через 4 секунды
    setTimeout(() => { progress.textContent = ''; }, 4000);
  }
};
