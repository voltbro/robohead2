

# Документация: ROS2 нода `silero_tts`

## Общее описание

Нода `silero_tts` выполняет офлайн-синтез речи (Text-to-Speech) на основе модели **Silero TTS v5_3_ru**. Модель загружается локально через `torch.package` — подключение к интернету **не требуется**. Поддерживается 5 русских голосов, автоматическая расстановка ударений и буквы «ё». Сгенерированный WAV-файл может быть сохранён на диск/в RAM и опционально воспроизведён через `media_driver`.

---

## Сервисы

### `<namespace>/speak`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/Speak` |
| **Имя по умолчанию** | `speak` (настраивается параметром `srv_speak_name`) |

#### Запрос

| Поле | Тип | Описание |
|------|-----|----------|
| `text` | `string` | Текст для озвучки. Для ручного ударения — символ `+` перед ударной гласной (например, `зам+ок`) |
| `voice` | `int16` | Индекс голоса (0–4, см. таблицу) |
| `path_to_save` | `string` | Путь для сохранения WAV-файла. Пустая строка — сохранение в `default_dir` (`/dev/shm/tmp_silero_tts.wav`) |
| `put_accent` | `bool` | `true` — автоматическая расстановка ударений моделью |
| `put_yo` | `bool` | `true` — автоматическая замена `е` на `ё` где необходимо |
| `play` | `bool` | `true` — после синтеза воспроизвести WAV через `media_driver` |

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Код результата |

#### Коды ответа

| Код | Значение |
|-----|----------|
| `0` | Успех |
| `-1` | Пустой текст |
| `-2` | Недопустимый индекс голоса (вне диапазона 0–4) |
| `-3` | Ошибка синтеза (исключение в Silero) |
| `-4` | Синтез вернул `None` (ошибка записи файла или создания директории) |
| `-5` | `play=true`, но параметр `srv_play_media` не настроен |
| `-6` | Ошибка воспроизведения через `media_driver` |

#### Доступные голоса

| Индекс (`voice`) | Имя | Пол |
|-------------------|-----|-----|
| `0` | `aidar` | Мужской |
| `1` | `baya` | Женский |
| `2` | `kseniya` | Женский |
| `3` | `eugene` | Мужской |
| `4` | `xenia` | Женский |

---

## Параметры конфигурации

Файл: `config/silero_tts.yaml`

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `model_path` | `string` | `"v5_3_ru.pt"` | Имя файла модели (относительно `<package>/model/`). |
| `device` | `string` | `"cpu"` | Устройство PyTorch (`"cpu"` или `"cuda"`) |
| `sample_rate` | `int` | `48000` | Частота дискретизации выходного аудио (8000 / 24000 / 48000) |
| `default_dir` | `string` | `"/dev/shm"` | Директория для WAV-файлов, когда `path_to_save` пустой |
| `srv_speak_name` | `string` | `"speak"` | Имя сервиса синтеза речи |
| `srv_play_media` | `string` | `""` | Полное имя сервиса `media_driver/play_media`. Пустая строка — воспроизведение через `play=true` невозможно |

---

## Модель

Модель хранится локально в директории `<package>/model/`. Загрузка из интернета **не выполняется** при работе ноды.

### Скачивание модели (однократно, требуется интернет)

```bash
cd <package>/model/
./download_model.sh
```

### Расположение

```
silero_tts/
└── model/
    ├── download_model.sh
    └── v5_3_ru.pt         # ~100 MB
```

---

## Управление ударениями

| Режим | `put_accent` | Формат текста | Пример |
|-------|-------------|---------------|--------|
| Автоматический | `true` | Обычный текст | `"Красивый замок на горе"` |
| Ручной | `false` | `+` перед ударной гласной | `"Красивый з+амок на гор+е"` |
| Комбинированный | `true` | `+` для исправления авто-ударений | `"Он открыл зам+ок"` |

---

## Примеры использования

### Из командной строки

```bash
# Синтез в RAM без воспроизведения
ros2 service call /silero_tts/speak robohead_interfaces/srv/Speak \
  "{text: 'Привет', voice: 4, path_to_save: '', put_accent: true, put_yo: true, play: false}"


# Синтез + воспроизведение через media_driver
ros2 service call /silero_tts/speak robohead_interfaces/srv/Speak \
  "{text: 'Озвучиваю и воспроизвожу.', voice: 4, path_to_save: '', put_accent: true, put_yo: true, play: true}"

# Сохранить на диск
ros2 service call /silero_tts/speak robohead_interfaces/srv/Speak \
  "{text: 'Сохраняю файл.', voice: 2, path_to_save: '/home/pi/speech.wav', put_accent: true, put_yo: true, play: false}"

# Ручное ударение
ros2 service call /silero_tts/speak robohead_interfaces/srv/Speak \
  "{text: 'Он открыл з+амок или зам+ок', voice: 4, path_to_save: '', put_accent: false, put_yo: false, play: true}"

# Мужской голос (aidar)
ros2 service call /silero_tts/speak robohead_interfaces/srv/Speak \
  "{text: 'Мужской голос.', voice: 0, path_to_save: '', put_accent: true, put_yo: true, play: true}"

# Прослушать сгенерированный файл вручную
aplay /dev/shm/tmp_silero_tts.wav
```
> После каждой перегенерации в `default_dir` файл `tmp_silero_tts.wav` перезаписывается!

## Сборка и запуск

```bash
# Скачать модель (однократно)
cd <package>/model/
./download_model.sh

# Сборка
colcon build --symlink-install --packages-select robohead_interfaces silero_tts

# Запуск
ros2 launch silero_tts silero_tts.launch.py
```

### Из другой ROS2 ноды (Python)
#TODO

### Из другой ROS2 ноды (C++)

#TODO

---
