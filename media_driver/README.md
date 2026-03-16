

# Документация: ROS2 нода `media_driver`

## Общее описание

Нода `media_driver` управляет воспроизведением медиаконтента (видео, изображения, аудио, интернет-потоки) на робоголове. Внутри используются **два независимых экземпляра MPV**:

- **Video player** — отображает видео/изображения на дисплее через DRM (без X-сервера), аудиодорожка отключена.
- **Audio player** — воспроизводит аудио через ALSA, видеовыход отключен.

Такое разделение позволяет **одновременно** показывать изображение/видео на экране и воспроизводить аудиофайл или интернет-радио. Дополнительно нода принимает ROS-изображения по топику и отображает их на дисплее в реальном времени (режим стриминга).

---

## Сервисы

### 1. `<namespace>/play_media`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/PlayMedia` |
| **Имя по умолчанию** | `play_media` (настраивается параметром `srv_play_media_name`) |

#### Запрос

| Поле | Тип | Описание |
|------|-----|----------|
| `path_to_video_file` | `string` | Путь к видео/изображению, URL потока или `__STOP__` для остановки. Пустая строка — не трогать видеоплеер |
| `path_to_audio_file` | `string` | Путь к аудиофайлу, URL потока или `__STOP__` для остановки. Пустая строка — не трогать аудиоплеер |
| `loop` | `bool` | Зацикливать воспроизведение (`true` — бесконечно, `false` — однократно) |

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Код результата |

#### Коды ответа

| Код | Значение |
|-----|----------|
| `0` | Успех |
| `-1` | Ошибка (файл не найден, неподдерживаемый формат, сбой воспроизведения) |

#### Поведение

- **Пустая строка** (`""`) в `path_to_video_file` или `path_to_audio_file` — соответствующий плеер не затрагивается.
- **Стоп-команда** (`__STOP__`, настраивается параметром `stop_command`) — останавливает соответствующий плеер.
- **Видеоплеер** принимает файлы видео и изображений. Изображение отображается статически (до 24 часов или до замены).
- **Аудиоплеер** принимает аудиофайлы и видеофайлы (воспроизводит только звуковую дорожку).
- Формат определяется по **расширению файла**.
- Поддерживаются **интернет-потоки** (HTTP, HTTPS, RTSP, RTMP и др.).
- При загрузке нового файла предыдущее воспроизведение **автоматически останавливается**.

#### Поддерживаемые форматы

| Категория | Расширения |
|-----------|-----------|
| **Изображения** | `.png`, `.jpg`, `.jpeg`, `.bmp`, `.webp`, `.gif`, `.tiff`, `.tif`, `.svg`, `.heic`, `.heif` |
| **Видео** | `.mp4`, `.mov`, `.avi`, `.mkv`, `.webm`, `.flv`, `.wmv`, `.m4v`, `.mpeg`, `.mpg`, `.3gp`, `.ts`, `.mxf` |
| **Аудио** | `.mp3`, `.wav`, `.ogg`, `.flac`, `.aac`, `.m4a`, `.wma`, `.opus`, `.aiff`, `.aif`, `.midi`, `.mid`, `.amr` |

#### Поддерживаемые протоколы потоков

`http://`, `https://`, `rtsp://`, `rtmp://`, `mms://`, `mmsh://`, `mmst://`, `mmsu://`, `hls://`, `dash://`, `ytdl://`

---

### 2. `<namespace>/set_volume`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |
| **Имя по умолчанию** | `set_volume` (настраивается параметром `srv_set_volume_name`) |

#### Запрос

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Желаемая громкость (0–100) |

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Установленная громкость или `-1` при ошибке |

> Значение ограничивается диапазоном `[0, 100]` автоматически.

---

### 3. `<namespace>/get_volume`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |
| **Имя по умолчанию** | `get_volume` (настраивается параметром `srv_get_volume_name`) |

#### Запрос

Значение поля `data` игнорируется.

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Текущая громкость аудиоплеера (0–100) или `-1` при ошибке |

---

### 4. `<namespace>/is_idle/audio`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |
| **Имя по умолчанию** | `is_idle/audio` (настраивается параметром `srv_is_idle_audio_name`) |

#### Запрос

Значение поля `data` игнорируется.

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | `1` — аудиоплеер простаивает (idle, пауза, EOF, ничего не загружено); `0` — воспроизведение активно |

---

### 5. `<namespace>/is_idle/display`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |
| **Имя по умолчанию** | `is_idle/display` (настраивается параметром `srv_is_idle_display_name`) |

#### Запрос

Значение поля `data` игнорируется.

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | `1` — видеоплеер простаивает; `0` — воспроизведение/отображение активно |

#### Логика определения idle

Плеер считается idle, если выполняется **любое** из условий:
- Свойство MPV `idle-active = yes` (ничего не воспроизводится)
- Воспроизведение на паузе (`pause = yes`)
- Достигнут конец файла (`eof-reached = yes`)
- Не загружен ни один файл (`path` пуст)

---

## Топики

### Подписки

### `<namespace>/stream`

| Поле | Значение |
|------|----------|
| **Тип** | `sensor_msgs/msg/Image` |
| **Имя по умолчанию** | `stream` (настраивается параметром `topic_stream_name`) |
| **QoS** | глубина очереди 1, `best_effort` |

#### Описание

Принимает ROS-изображения и отображает их на дисплее в реальном времени. Используется для вывода сгенерированных кадров (например, анимации глаз, интерфейса).

**Механизм работы:**
1. Входящее изображение конвертируется из ROS `Image` (формат `bgr8`) в OpenCV `Mat`.
2. Кадр сохраняется во временный файл `/dev/shm/robohead_stream_frame.ppm` (в оперативной памяти, без записи на диск).
3. Видеоплеер обновляет отображение через `loadfile ... replace`.

> **Важно:** Стриминг обновляет только видеоплеер и **не влияет** на аудиоплеер.

---

## Параметры конфигурации

Файл: `config/media_driver.yaml`

### Имена сервисов

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `srv_play_media_name` | `string` | `"play_media"` | Имя сервиса воспроизведения |
| `srv_set_volume_name` | `string` | `"set_volume"` | Имя сервиса установки громкости |
| `srv_get_volume_name` | `string` | `"get_volume"` | Имя сервиса получения громкости |
| `srv_is_idle_audio_name` | `string` | `"is_idle/audio"` | Имя сервиса проверки состояния аудио |
| `srv_is_idle_display_name` | `string` | `"is_idle/display"` | Имя сервиса проверки состояния дисплея |

### Имена топиков

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `topic_stream_name` | `string` | `"stream"` | Имя топика для приёма кадров видеопотока |

### Медиа-настройки

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `display_rotate` | `string` | `"270"` | Угол поворота изображения на дисплее (0, 90, 180, 270) |
| `default_volume` | `double` | `50.0` | Начальная громкость аудиоплеера (0–100) |
| `stop_command` | `string` | `"__STOP__"` | Строка-команда для остановки плеера в полях `path_to_video_file` / `path_to_audio_file` |

---

## Архитектура MPV-плееров

#TODO переделать картинку

```
┌─────────────────────────────────────────────┐
│               media_driver node             │
│                                             │
│  ┌───────────────────┐  ┌────────────────┐  │
│  │  Video Player     │  │  Audio Player  │  │
│  │  vo=drm           │  │  vo=null       │  │
│  │  ao=null (без     │  │  ao=alsa       │  │
│  │  звука)           │  │  (без видео)   │  │
│  │  hwdec=auto       │  │                │  │
│  │  keep-open=yes    │  │  keep-open=yes │  │
│  │  idle=yes         │  │  idle=yes      │  │
│  └───────────────────┘  └────────────────┘  │
│         ▲                      ▲            │
│         │                      │            │
│    play_media            play_media         │
│    stream topic          set/get_volume     │
└─────────────────────────────────────────────┘
```

---

## Примеры взаимодействия


## Сборка

```bash
colcon build --symlink-install --packages-select robohead_interfaces media_driver
```

## Запуск

```bash
ros2 launch media_driver media_driver.launch.py
```
Вывод после при успешном запуске:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [media_driver_node-1]: process started with pid [15077]
[media_driver_node-1] [INFO] [1773676180.823708121] [media_driver.media_driver]: [mpv_video] MPV player initialized successfully (type: VIDEO)
[media_driver_node-1] [INFO] [1773676180.830781476] [media_driver.media_driver]: [audio_mpv] MPV player initialized successfully (type: AUDIO)
[media_driver_node-1] [INFO] [1773676180.852060854] [media_driver.media_driver]: INITED
```

### Из командной строки

```bash
# Пути к файлам могут отличаться на вашем устройстве!

# Воспроизвести аудиофайл
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.mp3', loop: false}"

# Показать видео на дисплее (без зацикливания)
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '', loop: false}"

# Показать видео на дисплее (c зацикливанием)
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '', loop: true}"

# Показать изображение на дисплее
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.png', path_to_audio_file: '', loop: false}"

# Одновременно: видео на дисплее + аудио (зацикленное)
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.mp3', loop: true}"

# Остановить только аудио (видео продолжает играть)
# Предварительно запустите воспроизведение аудио и видео
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '__STOP__', loop: false}"

# Остановить всё воспроизведение
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '__STOP__', path_to_audio_file: '__STOP__', loop: false}"

# Воспроизвести интернет-радио
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: 'http://chanson.hostingradio.ru:8041/chanson256.mp3', loop: false}"


# Установить громкость 75%
ros2 service call /media_driver/set_volume robohead_interfaces/srv/SimpleCommand "{data: 75}"

# Узнать текущую громкость
ros2 service call /media_driver/get_volume robohead_interfaces/srv/SimpleCommand "{data: 0}"

# Проверить, завершено ли воспроизведение аудио
ros2 service call /media_driver/is_idle/audio robohead_interfaces/srv/SimpleCommand "{data: 0}"

# Проверить, завершено ли воспроизведение видео
ros2 service call /media_driver/is_idle/display robohead_interfaces/srv/SimpleCommand "{data: 0}"
```

### Из другой ROS2 ноды (C++)

#TODO

### Из другой ROS2 ноды (Python)

#TODO