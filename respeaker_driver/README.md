# Документация: ROS2 нода `respeaker_driver`

## Общее описание

Нода `respeaker_driver` обеспечивает взаимодействие с микрофонными массивами **ReSpeaker** (модели на базе **XVF3000** и **XVF3800**) от Seeed Studio. Нода выполняет три основные функции:

1. **Захват аудио** — многоканальная запись звука через PortAudio с публикацией в ROS-топики (основной канал + все каналы по отдельности).
2. **Определение направления звука (DOA)** — чтение угла направления источника звука с устройства по USB и публикация в топик.
3. **Управление LED-кольцом** — установка режимов подсветки, яркости, цвета через сервисы и топики.

Нода автоматически определяет модель ReSpeaker (XVF3000 или XVF3800) и количество аудиоканалов. Используется **MultiThreadedExecutor** для параллельной обработки.

---

## Поддерживаемые устройства

| Модель | Чип | VID | PID | Каналы | LED | 
|--------|-----|-----|-----|--------|-----|
| ReSpeaker Mic Array v2.0 | XVF3000 | `0x2886` | `0x0018` | до 6 | 12 шт. | 
| ReSpeaker USB Mic Array | XVF3800 | `0x2886` | `0x001A` | до 6 | 12 шт. |

При `vendor_id = 0` и `product_id = 0` в конфиг-файле устройство определяется автоматически (сначала проверяется XVF3800, затем XVF3000).

---

## Топики

### Публикуемые

### `<namespace>/audio/main`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/msg/AudioData` |
| **Имя по умолчанию** | `audio/main` (настраивается параметром `ros.topic_name.audio_main`) |
| **QoS** | глубина очереди 10 |

Аудиоданные **основного канала** (номер задаётся параметром `audio.main_channel`). Формат: массив `int16` длиной `frames_per_buffer`.

---

### `<namespace>/audio/channel_<N>`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/msg/AudioData` |
| **Имя по умолчанию** | `audio/channel_0` … `audio/channel_5` |
| **QoS** | глубина очереди 10 |

Аудиоданные **каждого канала** микрофонного массива. Количество активных топиков определяется числом каналов устройства (или параметром `audio.count_of_channels`). Максимум — 6 каналов.

---

### `<namespace>/doa`

| Поле | Значение |
|------|----------|
| **Тип** | `std_msgs/msg/Int32` |
| **Имя по умолчанию** | `doa` (настраивается параметром `ros.topic_name.doa`) |
| **QoS** | глубина очереди 10 |

Угол направления источника звука (**Direction of Arrival**) в градусах. Публикуется при каждом заполнении аудиобуфера (частота ≈ `sample_rate / frames_per_buffer` Гц).

Значение корректируется с учётом параметра `doa_yaw_offset` (поворот системы координат).

#TODO картинка с углами DOA

---

### Подписки

### `<namespace>/set_color_manual`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/msg/ColorArray` |
| **Имя по умолчанию** | `set_color_manual` (настраивается параметром `ros.topic_name.set_color_manual`) |
| **QoS** | глубина очереди 10 |

Ручная установка цвета **каждого светодиода** индивидуально. Сообщение должно содержать массив из **ровно 12** цветов (`Color[]`). При несовпадении количества — предупреждение, команда игнорируется.

#### Формат `Color`

| Поле | Тип | Описание |
|------|-----|----------|
| `red` | `uint8` | Красный (0–255) |
| `green` | `uint8` | Зелёный (0–255) |
| `blue` | `uint8` | Синий (0–255) |

---

## Сервисы

### 1. `<namespace>/set_mode`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |
| **Имя по умолчанию** | `set_mode` (настраивается параметром `ros.service_name.set_mode`) |

Установка режима работы LED-кольца.
На запись/воспроизведенеи звука никак не влияют. Являются просто световыми эффектами.

#### Запрос

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Номер режима (0–5) |

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | `0` — успех, `-1` — ошибка |

#### Режимы по моделям

| Код | XVF3000 | XVF3800 |
|-----|---------|---------|
| `0` | Выключить LED | Выключить LED |
| `1` | Trace (направление звука) | DOA (направление звука) |
| `2` | Listen (прослушивание) | Rainbow (радуга) |
| `3` | Wait (ожидание) | Single color (один цвет) |
| `4` | Speak (речь) | Breathe (дыхание) |
| `5` | Spin (вращение) | Ring (кольцо) |

#TODO описать поведения разных эффектов, приложить фото/видео

---

### 2. `<namespace>/set_brightness`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |
| **Имя по умолчанию** | `set_brightness` (настраивается параметром `ros.service_name.set_brightness`) |

Установка яркости LED-кольца.

#### Запрос

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | Яркость |

#### Допустимые значения

| Модель | Диапазон |
|--------|----------|
| XVF3000 | 0–31 |
| XVF3800 | 0–255 |

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | `0` — успех, `-1` — ошибка |

---

### 3. `<namespace>/set_color_all`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/Color` |
| **Имя по умолчанию** | `set_color_all` (настраивается параметром `ros.service_name.set_color_all`) |

Установка одного цвета для **всех** 12 светодиодов одновременно.

#### Запрос

| Поле | Тип | Описание |
|------|-----|----------|
| `red` | `uint8` | Красный (0–255) |
| `green` | `uint8` | Зелёный (0–255) |
| `blue` | `uint8` | Синий (0–255) |

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | `0` — успех, `-1` — ошибка |

---

### 4. `<namespace>/set_color_palette`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/ColorPalette` |
| **Имя по умолчанию** | `set_color_palette` (настраивается параметром `ros.service_name.set_color_palette`) |

Установка **двухцветной палитры** для анимационных режимов LED (DOA, breathe и т.д.).

#### Запрос

| Поле | Тип | Описание |
|------|-----|----------|
| `color_a` | `Color` | Первый цвет палитры (RGB) |
| `color_b` | `Color` | Второй цвет палитры (RGB) |

#### Ответ

| Поле | Тип | Описание |
|------|-----|----------|
| `data` | `int16` | `0` — успех, `-1` — ошибка |

---

## Параметры конфигурации

Файл: `config/respeaker_driver.yaml`

### USB

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `usb.vendor_id` | `int` | `0` | Vendor ID устройства. `0` — автоопределение (0x2886) |
| `usb.product_id` | `int` | `0` | Product ID устройства. `0` — автоопределение (перебор XVF3800 → XVF3000) |
| `usb.timeout` | `int` | `5000` | Таймаут USB-передачи данных (мс) |
| `usb.sleep_reset` | `int` | `10000` | Время ожидания после USB reset для переинициализации (мс) |
| `usb.sleep_stop` | `int` | `100` | Время ожидания при завершении работы ноды (мс) |

### Аудио

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `audio.sample_rate` | `int` | `16000` | Частота дискретизации (Гц) |
| `audio.frames_per_buffer` | `int` | `1024` | Размер аудиобуфера (фреймов). Определяет частоту публикации: `sample_rate / frames_per_buffer` Гц |
| `audio.count_of_channels` | `int` | `0` | Количество каналов. `0` — автоопределение (макс. 6) |
| `audio.main_channel` | `int` | `0` | Номер основного канала (0-индексация) |
| `audio.device_name_primary` | `string` | `"ReSpeaker"` | Имя аудиоустройства для поиска в PortAudio |
| `audio.device_name_fallback` | `string` | `"Mic Array"` | Запасное имя (после USB reset имя может измениться) |

### DOA

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `doa_yaw_offset` | `double` | `0.0` | Смещение нулевого положения DOA (градусы). `>0` — против часовой стрелки, `<0` — по часовой |

### Имена топиков

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `ros.topic_name.audio_main` | `string` | `"audio/main"` | Топик основного аудиоканала |
| `ros.topic_name.audio_channel_0` … `audio_channel_5` | `string` | `"audio/channel_0"` … `"audio/channel_5"` | Топики индивидуальных каналов |
| `ros.topic_name.doa` | `string` | `"doa"` | Топик угла направления звука |
| `ros.topic_name.set_color_manual` | `string` | `"set_color_manual"` | Топик для ручного управления LED |

### Имена сервисов

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `ros.service_name.set_mode` | `string` | `"set_mode"` | Сервис установки режима LED |
| `ros.service_name.set_brightness` | `string` | `"set_brightness"` | Сервис установки яркости |
| `ros.service_name.set_color_all` | `string` | `"set_color_all"` | Сервис установки цвета всех LED |
| `ros.service_name.set_color_palette` | `string` | `"set_color_palette"` | Сервис установки палитры |

---

### Поток данных

1. **PortAudio callback** заполняет буфер -> публикует аудиоданные по каналам -> вызывает `onAudioFrame()`.
2. **onAudioFrame()** читает DOA-угол с устройства по USB -> публикует в топик `doa`.
3. **Сервисы/топики LED** передают команды в `UsbHandler` -> USB control transfer на устройство.

---



## Процедура инициализации

```
1. Объявление параметров
2. USB: автоопределение устройства (XVF3800 → XVF3000)
3. USB: открытие устройства (libusb)
4. Audio: инициализация PortAudio
5. Audio: поиск аудиоустройства по имени
6. (если не найдено) → USB reset → повторный поиск
7. Audio: определение количества каналов
8. Audio: создание ROS-паблишеров
9. Audio: открытие аудиопотока (PortAudio callback)
10. ROS: создание сервисов и подписок для LED
11. Готово → "INITED [xvf3000/xvf3800] channels=N"
```

---

## Сборка

```bash
colcon build --symlink-install --packages-select robohead_interfaces respeaker_driver
```

## Запуск

```bash
ros2 launch respeaker_driver respeaker_driver.launch.py 
```
Вывод при успешном запуске:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [respeaker_driver_node-1]: process started with pid [4938]
[respeaker_driver_node-1] [INFO] [1773678691.497532071] [respeaker_driver.respeaker_driver]: Detected: xvf3800  VID=0x2886  PID=0x001A
[respeaker_driver_node-1] [WARN] [1773678691.744992526] [respeaker_driver.respeaker_driver]: Audio device 'ReSpeaker' not found. Trying USB reset...
[respeaker_driver_node-1] [INFO] [1773678702.247496844] [respeaker_driver.respeaker_driver]: Found audio device: reSpeaker XVF3800 4-Mic Array: USB Audio (hw:1,0) (2 channels)
[respeaker_driver_node-1] [INFO] [1773678702.247573066] [respeaker_driver.respeaker_driver]: Auto-detected 2 audio channels
[respeaker_driver_node-1] [INFO] [1773678702.274158191] [respeaker_driver.respeaker_driver]: INITED [xvf3800]  channels=2
```

---

## Примеры взаимодействия

### Из командной строки
#TODO проверить работу с LED на двух версиях мирокофнов
```bash
# Включить режим LED "trace" (XVF3000) / "DOA" (XVF3800)
ros2 service call /respeaker_driver/set_mode robohead_interfaces/srv/SimpleCommand "{data: 1}"

# Выключить LED
ros2 service call /respeaker_driver/set_mode robohead_interfaces/srv/SimpleCommand "{data: 0}"

# Установить яркость (XVF3000: 0-31, XVF3800: 0-255)
ros2 service call /respeaker_driver/set_brightness robohead_interfaces/srv/SimpleCommand "{data: 15}"

# Установить все LED в красный цвет
ros2 service call /respeaker_driver/set_color_all robohead_interfaces/srv/Color \
  "{red: 255, green: 0, blue: 0}"

# Установить каждый LED в отдельный цвет
ros2 topic pub /respeaker_driver/set_color_manual robohead_interfaces/msg/ColorArray 'colors: [
{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},

{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},

{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0},
{"red":255, "green":255, "blue":0}
]'

# Установить палитру (зелёный + синий)
ros2 service call /respeaker_driver/set_color_palette robohead_interfaces/srv/ColorPalette \
  "{color_a: {red: 0, green: 255, blue: 0}, color_b: {red: 0, green: 0, blue: 255}}"

# Прослушать DOA-угол
ros2 topic echo /respeaker_driver/doa

# Прослушать основной аудиоканал
ros2 topic echo /respeaker_driver/audio/main

# Проверить частоту публикации аудио
ros2 topic hz /respeaker_driver/audio/main
```
---



## Утилита `audio_to_wav.py`

Вспомогательный скрипт для записи аудио из ROS2 топика в WAV-файл. Полезен для отладки, проверки качества звука и тестирования микрофонного массива.

### Принцип работы

Скрипт подписывается на указанный аудиотопик, накапливает семплы в течение заданного времени и сохраняет результат в WAV-файл (16 бит, моно). После записи автоматически завершается.

### Параметры

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `topic` | `string` | `"audio/main"` | Имя топика с аудиоданными |
| `filename` | `string` | `"output.wav"` | Имя выходного WAV-файла |
| `duration` | `double` | `5.0` | Длительность записи (секунды) |
| `sample_rate` | `int` | `16000` | Частота дискретизации (Гц). Должна совпадать с настройкой `respeaker_driver` |

### Примеры использования

```bash
# Запись 5 секунд из основного канала (параметры по умолчанию)
ros2 run respeaker_driver audio_to_wav.py
# Аудио-файл будет записан по пути /home/pi/output.wav
# Запись идет 5 секунд с топика /respeaker_driver/audio/main
# Если запись "зависла", проверьте, есть ли в топике данные

# Запись 10 секунд из конкретного канала в указанный файл
ros2 run respeaker_driver audio_to_wav.py --ros-args \
  -p topic:=/respeaker_driver/audio/channel_0 \
  -p filename:=channel_0.wav \
  -p duration:=10.0
# Файл запишется по пути /home/pi/channel_0.wav

# Запись основного канала с указанием sample_rate
ros2 run respeaker_driver audio_to_wav.py --ros-args \
  -p topic:=/respeaker_driver/audio/main \
  -p filename:=test.wav \
  -p duration:=3.0 \
  -p sample_rate:=16000
```

### Прослушивание результата

```bash
# Через ALSA
aplay output.wav

# Через FFmpeg
ffplay -autoexit output.wav
```

---

> **Примечание:** Для доступа к USB-устройству без прав root необходимо настроить udev-правила:
> ```bash
> sudo tee /etc/udev/rules.d/99-respeaker.rules << 'EOF'
> SUBSYSTEM=="usb", ATTR{idProduct}=="0018", ATTR{idVendor}=="2886", MODE:="0666"
> SUBSYSTEM=="usb", ATTR{idProduct}=="001a", ATTR{idVendor}=="2886", MODE:="0666"
> EOF
> sudo udevadm control --reload-rules
> sudo udevadm trigger
> ```
> После этого переподключите устройство или перезагрузите систему.