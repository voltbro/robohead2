# Документация: ROS2 пакет `media_driver`

## Общее описание

Пакет `media_driver` обеспечивает управление мультимедийным контентом на робоголове через **два независимых экземпляра MPV-плеера**:

- **Video player** — отображает видео/изображения на дисплее через DRM (без X-сервера), аудиодорожка отключена
- **Audio player** — воспроизводит аудиофайлы через ALSA, видеовыход отключен

Такая архитектура позволяет **одновременно** показывать визуальный контент на экране и воспроизводить звук. Дополнительно поддерживается потоковый вывод ROS-изображений на дисплей в реальном времени.

Также пакет содержит узел **`touch_publisher`** для чтения событий **тачскрина** из `/dev/input/event*` (evdev, multitouch) и публикации их в ROS2-топик.

---

## Структура пакета

```
media_driver/
├── CMakeLists.txt      # Файл для CMake сборки пакета
├── config
│   └── media_driver.yaml   # Конфиг-файл
├── examples
│   ├── audio.aac       # Файлы-примеры аудио в разных форматах
│   ├── audio.aiff
│   ├── audio.flac
│   ├── audio.m4a
│   ├── audio.mp3
│   ├── audio.ogg
│   ├── audio.opus
│   ├── audio.wav
│   ├── audio.wma
│   ├── imager.py       # Скрипт для тестирования потокового вывода изображения с камеры на дисплей
│   ├── picture.bmp     # Файлы-примеры картинок в разных форматах
│   ├── picture.gif
│   ├── picture.jpeg
│   ├── picture.png
│   ├── picture.svg
│   ├── picture.tif
│   ├── picture.webp
│   ├── touchscreen.py  # Скрипт для тестирования работы тачскрина
│   ├── video.3gp       # Файлы-примеры видео в разных форматах
│   ├── video.avi
│   ├── video.flv
│   ├── video.mkv
│   ├── video.mov
│   ├── video.mp4
│   ├── video.mpeg
│   ├── video.ts
│   ├── video.webm
│   └── video.wmv
├── include             # Заголовочные файлы
│   └── media_driver
│       ├── mpv_player.hpp
│       ├── node.hpp
│       ├── touch_publisher.hpp
│       └── utils.hpp
├── launch              # Launch-фалы 
│   ├── media_driver.launch.py
│   └── touchscreen.launch.py
├── package.xml
├── README.md
└── src                 # Исходный код
    ├── main.cpp
    ├── mpv_player.cpp
    ├── node.cpp
    ├── touch_publisher.cpp
    └── utils.cpp
```

---

## Зависимости

### Системные библиотеки
- **libmpv** — медиаплеер
- **OpenCV** — обработка изображений
- **ALSA** — аудиовыход
- **Linux evdev** (`<linux/input.h>`) — чтение событий тачскрина из `/dev/input/event*`

### ROS2 пакеты
- `rclcpp` — клиентская библиотека C++
- `sensor_msgs` — стандартные сообщения для изображений
- `cv_bridge` — конвертация ROS ↔ OpenCV
- `robohead_interfaces` — пользовательские сервисы (`PlayMedia`, `SimpleCommand`)

---

## Сборка и запуск

### Сборка

```bash
cd ~/robohead_ws
colcon build --symlink-install --packages-select robohead_interfaces media_driver
source install/setup.bash
```

### Запуск

```bash
# Запускает узел работы с медиаплеером (дисплей+аудио) и узел работы с тачскрином:
ros2 launch media_driver media_driver.launch.py

# Запускает только узел работы с медиаплеером:
ros2 launch media_driver player.launch.py

# Запускает только узел работы с тачскрином:
ros2 launch media_driver touchscreen.launch.py

```
> При запуске используются параметры из конфиг-файла: `media_driver/config/media_driver.yaml`

**Ожидаемый вывод (при запуске обоих узлов):**
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [media_driver_node-1]: process started with pid [29242]
[INFO] [touch_publisher_node-2]: process started with pid [29243]
[touch_publisher_node-2] [INFO] [1776414096.540013704] [media_driver.touch_publisher_node]: Found: Waveshare  Waveshare -079-HD
[touch_publisher_node-2] [INFO] [1776414096.598018581] [media_driver.touch_publisher_node]: Touch range X:[0..4096] Y:[0..4096]
[touch_publisher_node-2] [INFO] [1776414096.598095618] [media_driver.touch_publisher_node]: Path: /dev/input/event4
[touch_publisher_node-2] [INFO] [1776414096.598231951] [media_driver.touch_publisher_node]: INITED
[media_driver_node-1] [INFO] [1776414096.664715001] [media_driver.media_driver]: [mpv_video] MPV player initialized successfully (type: VIDEO)
[media_driver_node-1] [INFO] [1776414096.672550045] [media_driver.media_driver]: [audio_mpv] MPV player initialized successfully (type: AUDIO)
[media_driver_node-1] [INFO] [1776414096.689966037] [media_driver.media_driver]: INITED
```

---

## API: Сервисы

### 1. `play_media` — Воспроизведение медиа

**Тип:** `robohead_interfaces/srv/PlayMedia`  
**Полное имя:** `/media_driver/play_media`

#### Поля запроса

| Поле | Тип | Описание |
|------|-----|----------|
| `path_to_video_file` | `string` | Путь к видео/изображению, URL потока или `__STOP__` (stop_command можно изменить в конфиг-файле) для остановки.<br/>Пустая строка — не трогать видеоплеер |
| `path_to_audio_file` | `string` | Путь к аудиофайлу, URL потока или `__STOP__` (stop_command можно изменить в конфиг-файле) для остановки.<br/>Пустая строка — не трогать аудиоплеер |
| `loop` | `bool` | `true` — зацикленное воспроизведение, `false` — однократное |

#### Поля ответа

| Поле | Тип | Значение |
|------|-----|----------|
| `data` | `int16` | `0` — успех<br/>`-1` — ошибка (файл не найден, неподдерживаемый формат) |

#### Поддерживаемые форматы

| Категория | Расширения |
|-----------|-----------|
| **Изображения** | `.png`, `.jpeg` (`.jpg`), `.bmp`, `.webp`, `.gif`, `.tiff` (`.tif`), `.svg` |
| **Видео** | `.mp4`, `.mov`, `.avi`, `.mkv`, `.webm`, `.flv`, `.wmv`, `.mpeg`, `.3gp`, `.ts`, `.m3u3` |
| **Аудио** | `.mp3`, `.wav`, `.ogg`, `.flac`, `.aac`, `.m4a`, `.wma`, `.opus`, `.aiff` |

#### Поддерживаемые протоколы

`http://`, `https://`

### Примеры

> Обратите внимание: пути до файлов могут отличаться на вашем устройстве

**Воспроизвести аудио (тест всех форматов)**
```bash
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.aac', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.aiff', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.flac', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.m4a', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.mp3', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.ogg', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.opus', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.wav', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.wma', loop: false}"
```

**Воспроизвести видео (тест всех форматов)**

```bash
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.3gp', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.3gp', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.avi', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.avi', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.flv', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.flv', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mkv', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mkv', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mov', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mov', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mpeg', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mpeg', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.ts', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.ts', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.webm', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.webm', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.wmv', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.wmv', loop: false}"
```

**Вывести картинку (тест всех форматов)**

```bash
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.bmp', path_to_audio_file: '', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.gif', path_to_audio_file: '', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.jpeg', path_to_audio_file: '', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.png', path_to_audio_file: '', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.svg', path_to_audio_file: '', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.tif', path_to_audio_file: '', loop: false}"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/picture.webp', path_to_audio_file: '', loop: false}"
```

**Комбинированные примеры:**
```bash
# Воспроизвести видео со звуком (без зацикливания)
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', loop: false}"

# Прекратить воспроизведение видео и звука, очистить экран
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '__STOP__', path_to_audio_file: '__STOP__', loop: false}"

# Воспроизвести видео без звука (без зацикливания). Если в момент вызова воспроизводился звук - он НЕ будет прерван.
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '', loop: false}"

# Воспроизвести видео без звука (без зацикливания). Если в момент вызова воспроизводился звук - он БУДЕТ прерван.
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '__STOP__', loop: false}"

# Воспроизвести интернет-радио
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: 'http://chanson.hostingradio.ru:8041/chanson256.mp3', loop: false}"

# Воспроизвести интернет-стрим
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: 'http://hls.mirtv.cdnvideo.ru/mirtv-parampublish/mir24_2500/playlist.m3u8', path_to_audio_file: 'http://hls.mirtv.cdnvideo.ru/mirtv-parampublish/mir24_2500/playlist.m3u8', loop: false}"
  

# Остановить только видео (аудио продолжит играть)
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '__STOP__', path_to_audio_file: '', loop: false}"

# Остановить только аудио
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '__STOP__', loop: false}"

# Зациклить аудио
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/audio.mp3', loop: true}"

# Зациклить видео+аудио
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia \
  "{path_to_video_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', path_to_audio_file: '/home/pi/robohead_ws/src/robohead2/media_driver/examples/video.mp4', loop: true}"
```

---

### 2. `set_volume` — Установка громкости

**Тип:** `robohead_interfaces/srv/SimpleCommand`  
**Полное имя:** `/media_driver/set_volume`

#### Поля запроса

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Желаемая громкость (0–100) |

#### Поля ответа

| Поле | Тип | Значение |
|------|-----|----------|
| `data` | `int16` | Установленная громкость или `-1` при ошибке |

При выходе из диапазона от 0 до 100 устанавливает наиболее близкое допустимое значение.

#### Пример

```bash
ros2 service call /media_driver/set_volume robohead_interfaces/srv/SimpleCommand "{data: 75}"
```

---

### 3. `get_volume` — Получение громкости

**Тип:** `robohead_interfaces/srv/SimpleCommand`  
**Полное имя:** `/media_driver/get_volume`

В поле запроса ничего задавать не нужно - значение игнорируется.

#### Поля ответа

| Поле | Тип | Значение |
|------|-----|----------|
| `data` | `int16` | Текущая громкость (0–100) или `-1` при ошибке |

#### Пример

```bash
ros2 service call /media_driver/get_volume robohead_interfaces/srv/SimpleCommand "{data: 0}"
```

---

### 4. `is_idle/audio` — Проверка состояния аудиоплеера

**Тип:** `robohead_interfaces/srv/SimpleCommand`  
**Полное имя:** `/media_driver/is_idle/audio`

В поле запроса ничего задавать не нужно - значение игнорируется.

#### Поля ответа

| Поле | Тип | Значение |
|------|-----|----------|
| `data` | `int16` | `1` — плеер простаивает (idle, пауза, EOF)<br/>`0` — активное воспроизведение |

#### Логика определения idle

Плеер считается idle, если выполняется **любое** условие:
- `idle-active = yes` (ничего не загружено)
- `pause = yes` (на паузе)
- `eof-reached = yes` (конец файла)
- `path` пуст (файл не загружен)

#### Пример

```bash
ros2 service call /media_driver/is_idle/audio robohead_interfaces/srv/SimpleCommand "{data: 0}"
```

---

### 5. `is_idle/display` — Проверка состояния видеоплеера

**Тип:** `robohead_interfaces/srv/SimpleCommand`  
**Полное имя:** `/media_driver/is_idle/display`

В поле запроса ничего задавать не нужно - значение игнорируется.

#### Поля ответа

| Поле | Тип | Значение |
|------|-----|----------|
| `data` | `int16` | `1` — дисплей простаивает<br/>`0` — активное отображение |

#### Пример

```bash
ros2 service call /media_driver/is_idle/display robohead_interfaces/srv/SimpleCommand "{data: 0}"
```

---

## API: Топики

### Подписка: `stream` — Потоковый вывод изображений

**Тип:** `sensor_msgs/msg/Image`  
**Полное имя:** `/media_driver/stream`  
**QoS:** глубина очереди 1, `best_effort`

#### Описание

Принимает ROS-изображения (формат `bgr8`) и отображает их на дисплее в реальном времени. Используется для вывода динамического контента (анимации лица, интерфейса).

#### Механизм работы

1. Входящее изображение конвертируется в OpenCV `Mat`
2. Кадр сохраняется в `/dev/shm/robohead_stream_frame.ppm` (RAM, без диска)
3. Видеоплеер обновляет отображение через `loadfile ... replace`

> **Важно:** Стриминг не влияет на аудиоплеер и не требует остановки текущего воспроизведения (оно остонавливается автоматически)

#### Пример
Запустите пакет usb_cam (появится топик `/image_raw`) - он получает данные с камеры и публикует их в топик
```bash
ros2 run usb_cam usb_cam_node_exe
```
В отдельном терминале перейдите в папку примеров пакета и запустите скрипт, пересылающий кадры из топика `/image_raw` в топик `/media_driver/stream`:
```bash
cd /home/pi/robohead_ws/src/robohead2/media_driver/examples/
python3 imager.py
```


### Паблишер: `touchscreen` - публикация касаний тачскрина

**Тип:** `robohead_interfaces/msg/TouchEvent`  
**Полное имя:** `/media_driver/touchscreen`  
**QoS:** глубина очереди 10

#### Поля сообщения `TouchEvent`
| Поле | Тип | Описание |
|------|-----|----------|
| `slot` | `int` | Номер multitouch-слота (из `ABS_MT_SLOT`) |
| `tracking_id` | `int` | Идентификатор касания (из `ABS_MT_TRACKING_ID`). На `up` остаётся последним валидным значением |
| `x` | `int` | Координата X (после поворота), в диапазоне устройства |
| `y` | `int` | Координата Y (после поворота), в диапазоне устройства |
| `state` | `string` | `"down"`, `"move"`, `"up"` |

#### Семантика событий
- `"down"` публикуется в момент касания пальца тачскрина (когда для слота получен валидный `tracking_id >= 0` и есть координаты `x/y`)
- `"move"` публикуется если палец двигается по тачскрину (только если координаты реально изменились относительно предыдущего опубликованного положения)
- `"up"` публикуется при отрыве пальца от тачскрина (когда приходит `tracking_id = -1` для слота, слот удаляется из внутренней карты состояний)

#### Поворот
Узел читает диапазоны координат из драйвера (`EVIOCGABS` для `ABS_MT_POSITION_X/Y`, иначе `ABS_X/Y`) и использует их для вычисления `width_`/`height_`.

Параметр `display_rotate` задаётся в градусах (обычно `0/90/180/270`).  
Внутри узла применяется компенсация поворота:

- берётся `r = display_rotate % 360`
- затем фактический поворот тача делается как `rotation_deg_ = (360 - r) % 360`

То есть если экран повернут, тач автоматически “поворачивается обратно”, чтобы итоговые `(x,y)` совпадали с координатами отображения.

Поддерживаются углы `0/90/180/270` (быстрые ветки) и произвольные углы (через sin/cos с поворотом вокруг центра и clamp в диапазон).

### Пример
Запустите пакет `media_driver` (плеер+тачскрин)
```bash
ros2 launch media_driver media_driver.launch.py
```
В отдельном терминале перейдите в папку примеров пакета и запустите скрипт:
```bash
cd /home/pi/robohead_ws/src/robohead2/media_driver/examples/
python3 touchscreen.py
```
При касании под пальцем будет появляться красный круг (down). При движении он сменит цвет на зеленый (move). При отрыве пальца от дисплея кружок станет синим (up).
Если каснуться несколькими пальцами, то кружочков будет несколько.
`s0`, `s1`,... - номера слотов (один слот - один палец). `id56`, `id57` - уникальный номер касания: остается одним и тем же, пока палец не оторван от экрана, при повторном касании инкрементируется на единицу.

---

## Параметры конфигурации

Файл: `media_driver/config/media_driver.yaml`

### Имена сервисов

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `srv_play_media_name` | `string` | `play_media` | Имя сервиса воспроизведения |
| `srv_set_volume_name` | `string` | `set_volume` | Имя сервиса установки громкости |
| `srv_get_volume_name` | `string` | `get_volume` | Имя сервиса получения громкости |
| `srv_is_idle_audio_name` | `string` | `is_idle/audio` | Имя сервиса проверки состояния аудио |
| `srv_is_idle_display_name` | `string` | `is_idle/display` | Имя сервиса проверки состояния дисплея |


### Имена топиков

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `topic_stream_name` | `string` | `stream` | Имя топика для приёма видеопотока |
| `topic_touchscreen_name` | `string` | `touchscreen` | Имя топика для публикации касаний тачскрина |

### Настройки медиа

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `display_rotate` | `string` | `270` | Угол поворота дисплея (0, 90, 180, 270) |
| `default_volume` | `double` | `50.0` | Начальная громкость аудиоплеера (0–100) |
| `stop_command` | `string` | `__STOP__` | Команда для остановки плеера |


### Настройки тачскрина

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `device_name` | `string` | `waveshare` | Подстрока для поиска устройства по имени (регистронезависимо). Сравнивается с `EVIOCGNAME` |
| `device_path` | `string` | `/dev/input/` | Путь, где искать `event*` |
| `display_rotate` | `string` | `270` | Угол поворота дисплея (0, 90, 180, 270) |

---

## Дополнительные ресурсы

- [Документация libmpv](https://mpv.io/manual/master/)
- [ROS2 сервисы](https://docs.ros.org/en/jazzy/Tutorials/Services/Understanding-ROS2-Services.html)
- [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)
- [evdev event-codes](https://www.kernel.org/doc/html/latest/input/event-codes.html#input-event-codes)