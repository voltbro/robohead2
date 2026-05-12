
#TODO
# Документация: ROS2 пакет `turtlebro_controller`

## Общее описание

Пакет `robohead_controller` — **центральный управляющий пакет** робоголовы. Он связывает все драйверы (уши, шея, дисплей, аудио, микрофон, камера, датчик батареи, распознавание речи) в единую систему с голосовым управлением и автоматическим поведением.

Контроллер реализует:
- **Подключение ко всем драйверам** через ROS2 сервисы и топики.
- **Голосовое управление** по схеме: wake phrase → распознавание команды → выполнение действия.
- **Систему действий (actions)** — модульные Python-скрипты, описывающие поведение робота.
- **Мониторинг батареи** с автоматической реакцией на низкий заряд.
- **Быстрые команды** — команды, выполняемые без wake phrase (громче, тише и т.д.).

---

## Архитектура

#TODO норм схема

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        robohead_controller                              │
│                                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────────────┐  │
│  │ Commander     │  │ BatteryMonitor│  │ ActionManager                │  │
│  │               │  │               │  │                             │  │
│  │ Очередь:      │  │ Проверяет     │  │ Загружает и выполняет       │  │
│  │ wake_phrases  │  │ напряжение    │  │ Python-скрипты действий     │  │
│  │ commands      │  │ каждые 0.5с   │  │ в изолированных потоках     │  │
│  │ fast_commands │  │               │  │                             │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┬──────────────┘  │
│         │                  │                         │                  │
│  ┌──────┴──────────────────┴─────────────────────────┴──────────────┐  │
│  │                      Driver Connectors                            │  │
│  │                                                                    │  │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌───────────────┐  │  │
│  │  │ MediaDriver│ │ EarsDriver │ │ NeckDriver │ │ SensorDriver  │  │  │
│  │  │ Connector  │ │ Connector  │ │ Connector  │ │ Connector     │  │  │
│  │  └────────────┘ └────────────┘ └────────────┘ └───────────────┘  │  │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌───────────────┐  │  │
│  │  │ Respeaker  │ │ ASR        │ │ KWS        │ │ UsbCam        │  │  │
│  │  │ Connector  │ │ Connector  │ │ Connector  │ │ Connector     │  │  │
│  │  └────────────┘ └────────────┘ └────────────┘ └───────────────┘  │  │
│  └────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
         │                                            │
         ▼                                            ▼
   ROS2 сервисы/топики                        Python-скрипты
   драйверов                                  actions/*.py
```

---

## Основной цикл работы

#TODO норм цикл

```
1. Запуск → инициализация всех коннекторов
2. connect_all_drivers() → ожидание доступности всех сервисов
3. Выполнение действия "std_startup" (приветствие)
4. Включение KWS (режим 1)
5. BatteryMonitor проверяет напряжение → разрешает/запрещает работу
6. При разрешении работы → действие "std_wait" (ожидание)

Основной цикл Commander (10 Гц):
  ├── Есть fast_command? → execute_action(fast_command, cancelling=False)
  ├── Состояние "wait_wake_phrase":
  │     └── Пришла wake phrase? → 
  │           ├── execute_action("std_attention")
  │           ├── ASR → mode=1 (Grammar)
  │           ├── KWS → mode=0 (Off)
  │           └── Переход в "wait_command"
  └── Состояние "wait_command":
        └── Пришла команда? →
              ├── Если команда (не таймаут) → execute_action(команда, on_complete="std_wait")
              ├── Если таймаут → execute_action("std_wait")
              ├── KWS → mode=1 (On)
              └── Переход в "wait_wake_phrase"
```

---

## Структура пакета

```
robohead_controller/
├── config/                           # Конфиги всех драйверов + контроллера
│   ├── robohead_controller.yaml      # Основной конфиг
│   ├── media_driver.yaml
│   ├── ears_driver.yaml
│   ├── neck_driver.yaml
│   ├── sensor_driver.yaml
│   ├── respeaker_driver.yaml
│   ├── speech_recognizer_asr.yaml
│   └── speech_recognizer_kws.yaml
├── launch/
│   ├── robohead_controller.launch.py # Полный запуск (драйверы + контроллер)
│   └── dependencies.launch.py       # Запуск только драйверов
├── robohead_controller/
│   ├── main.py                       # Точка входа
│   ├── controller.py                 # Главный класс RoboheadController
│   ├── core/
│   │   ├── action_manager.py         # Менеджер действий
│   │   ├── battery_monitor.py        # Мониторинг батареи
│   │   └── commander.py              # Обработчик голосовых команд
│   ├── drivers/                      # Коннекторы к драйверам
│   │   ├── ears_driver.py
│   │   ├── neck_driver.py
│   │   ├── media_driver.py
│   │   ├── sensor_driver.py
│   │   ├── respeaker_driver.py
│   │   ├── speech_recognizer_asr.py
│   │   ├── speech_recognizer_kws.py
│   │   └── usb_cam.py
│   ├── actions/                      # Действия (обычные)
│   │   ├── std_startup/action.py
│   │   ├── std_wait/action.py
│   │   ├── std_attention/action.py
│   │   ├── std_low_bat/action.py
│   │   ├── std_greeting/action.py
│   │   ├── std_ears/action.py
│   │   ├── std_ball_tracker/action.py
│   │   ├── std_make_photo/action.py
│   │   ├── std_left_ear/action.py
│   │   └── std_right_ear/action.py
│   └── fast_actions/                 # Быстрые действия
│       ├── std_volume_up/action.py
│       └── std_volume_down/action.py
```

---

## Компоненты ядра

### `main.py` — Точка входа

Создаёт экземпляр `RoboheadController`, подключает все драйверы, запускает стартовое действие и `MultiThreadedExecutor` для параллельной обработки колбэков.

### `controller.py` — Главный класс

Класс `RoboheadController` (наследник `rclpy.Node`):

- Хранит экземпляры всех коннекторов драйверов.
- Хранит очереди событий: `queue_wake_phrases`, `queue_commands`, `queue_fast_commands`.
- Предоставляет метод `sleep(cancel_event, duration)` для использования в действиях (с поддержкой отмены).
- Загружает маппинг `actions_match` из конфига (имя команды → путь к Python-скрипту).
- Флаг `is_allow_work` управляется `BatteryMonitor`.

### `action_manager.py` — Менеджер действий

Управляет запуском, отменой и изоляцией действий:

- **`execute_action(name, on_complete, cancelling)`** — запуск действия:
  - Если `cancelling=True` — отменяет текущее действие перед запуском нового.
  - Действие выполняется в **отдельном daemon-потоке**.
  - После успешного завершения может автоматически запустить действие `on_complete`.

- **Изоляция ошибок** — ошибки в действиях логируются, но не приводят к падению контроллера.
- **Отмена** — через `threading.Event`. Действие должно периодически проверять `cancel_event.is_set()`. Если `cancel_event.is_set()==True`, то действие должно завершиться.

### `battery_monitor.py` — Мониторинг батареи

Проверяет напряжение батареи каждые **0.5 секунды**:

| Условие | Действие |
|---------|----------|
| `voltage < low_voltage_threshold` и работа разрешена | Запрещает работу (`is_allow_work = False`), запускает `std_low_bat` |
| `voltage ≥ (threshold + hysteresis)` и работа запрещена | Разрешает работу (`is_allow_work = True`), запускает `std_wait` |

Гистерезис предотвращает частое переключение при напряжении вблизи порога.

### `commander.py` — Обработчик команд

Работает с частотой **10 Гц**, реализует конечный автомат:

#TODO сделать хорошую картинку конечного автомата

```
┌────────────────────┐        wake phrase        ┌─────────────────┐
│  wait_wake_phrase  │ ─────────────────────────▶ │  wait_command   │
│                    │                            │                 │
│  KWS: on           │        command/timeout     │  KWS: off       │
│  ASR: off          │ ◀───────────────────────── │  ASR: grammar   │
└────────────────────┘                            └─────────────────┘
```

**Быстрые команды** обрабатываются **в любом состоянии** без прерывания текущего действия (`cancelling=False`).

---

## Коннекторы драйверов

Каждый коннектор — обёртка над ROS2 клиентами/подписчиками с поддержкой `cancel_event` и блокирующего/неблокирующего режима.

### `EarsDriverConnector`

| Метод | Описание |
|-------|----------|
| `connect()` | Подключение к сервисам `ears_set_angle`, `is_idle` |
| `set_angle(cancel_event, left, right, duration, block)` | Установка углов ушей. `block=True` — ждать завершения движения |
| `is_idle(cancel_event)` | Проверка завершения движения |

### `NeckDriverConnector`

| Метод | Описание |
|-------|----------|
| `connect()` | Подключение к сервисам `neck_set_angle`, `is_idle` |
| `set_angle(cancel_event, vertical, horizontal, duration, block)` | Установка углов шеи. `block=True` — ждать завершения |
| `is_idle(cancel_event)` | Проверка завершения движения |

### `MediaDriverConnector`

| Метод | Описание |
|-------|----------|
| `connect()` | Подключение ко всем сервисам и топикам media_driver |
| `play_media(cancel_event, video_path, audio_path, loop, block)` | Воспроизведение видео + аудио |
| `play_audio(cancel_event, audio_path, loop, block)` | Воспроизведение только аудио |
| `play_display(cancel_event, video_path, loop, block)` | Отображение только видео/изображения |
| `set_volume(cancel_event, volume)` | Установка громкости (0–100) |
| `get_volume(cancel_event)` | Получение текущей громкости |
| `is_idle_audio(cancel_event)` | Проверка завершения аудио |
| `is_idle_display(cancel_event)` | Проверка завершения видео |
| `stream_publish(image_msg)` | Публикация кадра в стрим |

### `SensorDriverConnector`

| Метод/Поле | Описание |
|------------|----------|
| `connect()` | Подписка на топик `battery` |
| `battery_voltage` | Текущее напряжение (В) |
| `battery_current` | Текущий ток (А) |
| `battery_power_supply_status` | Статус питания |

### `RespeakerDriverConnector`

| Метод/Поле | Описание |
|------------|----------|
| `connect()` | Подключение к сервисам LED и подписка на DOA, аудио |
| `doa` | Текущий угол DOA (градусы) |
| `audio_main` | Последний аудиофрейм основного канала |

### `SpeechRecognizerAsrConnector`

| Метод | Описание |
|-------|----------|
| `connect()` | Подключение к сервису `set_mode` и подписка на `commands` |
| `set_mode(mode)` | Установка режима ASR (0/1/2) |
| Колбэк `commands` | Полученные команды добавляются в `controller.queue_commands` |

### `SpeechRecognizerKwsConnector`

| Метод | Описание |
|-------|----------|
| `connect()` | Подключение к сервису `set_mode` и подписка на `wake_phrases`, `fast_commands` |
| `set_mode(mode)` | Установка режима KWS (0/1) |
| Колбэк `wake_phrases` | Полученные фразы добавляются в `controller.queue_wake_phrases` |
| Колбэк `fast_commands` | Полученные команды добавляются в `controller.queue_fast_commands` |

### `UsbCamConnector`

| Метод/Поле | Описание |
|------------|----------|
| `connect()` | Подписка на топик `image_raw` |
| `image_raw` | Последний кадр с камеры (`sensor_msgs/msg/Image`) |

---

## Система действий (Actions)

### Структура действия

Каждое действие — Python-файл с функцией `run()`:

```python
def run(controller, action_name: str, cancel_event: threading.Event):
    """
    Args:
        controller: Ссылка на RoboheadController (доступ ко всем драйверам)
        action_name: Имя действия (строка). Совпадает с командой, которой был вызван этот action.
        cancel_event: threading.Event для проверки отмены
    """
    # Логика действия...
```

### Правила написания действий

1. **Периодически проверять `cancel_event.is_set()`** — для корректной отмены. Методы коннекторов драйверов делают внутри себя это за вас. Обращайте на это внимание при написании длительных операций (длительные циклы, time.sleep (не рекомендуется) и т.д.).
2. **Передавать `cancel_event`** во все вызовы драйверов.
3. **Не использовать `time.sleep()` напрямую** — использовать `controller.sleep(cancel_event, duration)` или собственную обёртку с проверкой отмены.
4. Ошибки в действиях **изолированы** — не приводят к падению контроллера.

### Стандартные действия

| Имя | Файл | Описание |
|-----|------|----------|
| `std_startup` | `actions/std_startup/action.py` | Стартовое приветствие: воспроизведение видео и аудио при запуске |
| `std_wait` | `actions/std_wait/action.py` | Режим ожидания: зацикленная анимация на дисплее, остановка аудио, уши и шея в нейтральное положение |
| `std_attention` | `actions/std_attention/action.py` | Реакция на wake phrase: анимация внимания, поворот головы к источнику звука (DOA), движение ушей |
| `std_low_bat` | `actions/std_low_bat/action.py` | Реакция на низкий заряд батареи |
| `std_ears` | `actions/std_ears/action.py` | Демонстрация ушей: анимация с поочерёдным движением ушей и покачиванием головы |
| `std_greeting` | `actions/std_greeting/action.py` | Приветствие по голосовой команде |
| `std_ball_tracker` | `actions/std_ball_tracker/action.py` | Слежение за шариком с камеры |
| `std_make_photo` | `actions/std_make_photo/action.py` | Фотографирование с камеры |
| `std_left_ear` | `actions/std_left_ear/action.py` | Демонстрация левого уха |
| `std_right_ear` | `actions/std_right_ear/action.py` | Демонстрация правого уха |

### Быстрые действия (Fast Actions)

Выполняются **без прерывания** текущего действия (`cancelling=False`). Используются для команд, которые должны сработать мгновенно.

| Имя | Файл | Описание |
|-----|------|----------|
| `громче` | `fast_actions/std_volume_up/action.py` | Увеличение громкости на 10. При достижении максимума — звуковое уведомление |
| `тише` | `fast_actions/std_volume_down/action.py` | Уменьшение громкости на 10 |

### Маппинг действий (`actions_match`)

Соответствие между именем команды и Python-скриптом задаётся в конфиге как JSON:

```yaml
actions_match: >
  {
    "std_startup":        "std_startup/action.py",
    "std_wait":           "std_wait/action.py",
    "поздоровайся":       "std_greeting/action.py",
    "покажи уши":         "std_ears/action.py",
    "громче":             "/absolute/path/to/action.py"
  }
```

- **Относительные пути** разрешаются от `robohead_controller/actions/`.
- **Абсолютные пути** используются как есть.
- **Ключи** должны совпадать с командами в `speech_recognizer_asr.yaml` (`commands`) и `speech_recognizer_kws.yaml` (`fast_commands`).

---

## Параметры конфигурации

Файл: `config/robohead_controller.yaml`

### Основные параметры

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `low_voltage_threshold` | `float` | `6.3` | Порог низкого напряжения батареи (В). Ниже — запуск `std_low_bat` |
| `low_voltage_hysteresis` | `float` | `0.5` | Гистерезис восстановления (В). Работа разрешается при `voltage ≥ threshold + hysteresis` |
| `wait_timeout` | `float` | `1.0` | Таймаут ожидания доступности сервисов при подключении (сек) |
| `actions_match` | `string` (JSON) | `"{}"` | JSON-маппинг: имя команды → путь к скрипту действия |

### Имена сервисов и топиков драйверов

Конфиг содержит полные имена (с namespace) всех сервисов и топиков для подключения к драйверам:

```yaml
media_driver:
  srv_play_media_name: "/robohead/media_driver/play_media"
  # ...

ears_driver:
  srv_ears_set_angle_name: "/robohead/ears_driver/ears_set_angle"
  # ...

neck_driver:
  srv_neck_set_angle_name: "/robohead/neck_driver/neck_set_angle"
  # ...

respeaker_driver:
  topic_name:
    doa: "/robohead/respeaker_driver/doa"
    # ...
  service_name:
    set_mode: "/robohead/respeaker_driver/set_mode"
    # ...

sensor_driver:
  topic_name: "/robohead/sensor_driver/battery"

speech_recognizer_asr:
  service_name:
    set_mode: "/robohead/speech_recognizer/asr/set_mode"
  topic_name:
    commands: "/robohead/speech_recognizer/asr/commands"

speech_recognizer_kws:
  service_name:
    set_mode: "/robohead/speech_recognizer/kws/set_mode"
  topic_name:
    wake_phrases: "/robohead/speech_recognizer/kws/wake_phrases"
    fast_commands: "/robohead/speech_recognizer/kws/fast_commands"

usb_cam:
  topic_name_image_raw: "/robohead/usb_cam/image_raw"
```

---

## Launch-файлы

### `robohead_controller.launch.py` — Полный запуск

Запускает **все драйверы** и **контроллер**:

```bash
ros2 launch robohead_controller robohead_controller.launch.py
```

Последовательность:
1. `dependencies.launch.py` — запуск всех драйверов.
2. `robohead_controller` — запуск контроллера.

### `dependencies.launch.py` — Только драйверы

Запускает все драйверы без контроллера (для отладки):

```bash
ros2 launch robohead_controller dependencies.launch.py
```

Запускаемые ноды (namespace `/robohead/`):

| Нода | Пакет | Namespace |
|------|-------|-----------|
| `media_driver_node` | `media_driver` | `/robohead/media_driver/` |
| `neck_driver` | `neck_driver` | `/robohead/neck_driver/` |
| `sensor_driver` | `sensor_driver` | `/robohead/sensor_driver/` |
| `ears_driver` | `ears_driver` | `/robohead/ears_driver/` |
| `usb_cam` | `usb_cam` | `/robohead/usb_cam/` |
| `respeaker_driver` | `respeaker_driver` | `/robohead/respeaker_driver/` |
| `speech_recognizer_kws_node` | `speech_recognizer` | `/robohead/speech_recognizer/kws/` |
| `speech_recognizer_asr_node` | `speech_recognizer` | `/robohead/speech_recognizer/asr/` |

---

## Пример создания нового действия
#TODO описать подробнее в документации 
1. Создать файл действия
2. Добавить в маппинг (`robohead_controller.yaml`)
3. Добавить команду в ASR (`speech_recognizer_asr.yaml`)
4. Пересобрать и запустить
```bash
colcon build --symlink-install --packages-select robohead_controller speech_recognizer
ros2 launch robohead_controller robohead_controller.launch.py
```

---

## Сборка

```bash
colcon build --symlink-install --packages-select robohead_interfaces robohead_controller
```

## Запуск

```bash
# Полный запуск (драйверы + контроллер)
ros2 launch robohead_controller robohead_controller.launch.py

# Только драйверы (для отладки)
ros2 launch robohead_controller dependencies.launch.py
```