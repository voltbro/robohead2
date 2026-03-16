

# Документация: ROS2 пакет `speech_recognizer`

## Общее описание

Пакет `speech_recognizer` обеспечивает офлайн-распознавание речи на основе библиотеки **Vosk**. Пакет содержит **две независимые ноды**, которые могут работать одновременно:

- **KWS (Keyword Spotting)** — непрерывное обнаружение ключевых фраз (wake words) и быстрых команд в аудиопотоке.
- **ASR (Automatic Speech Recognition)** — распознавание команд или произвольной речи по запросу (активируется через сервис).

Обе ноды получают аудио из топика `respeaker_driver` и используют локальные Vosk-модели (без подключения к интернету).

---

## Архитектура

```
                    ┌─────────────────────────┐
                    │    respeaker_driver      │
                    │  pub: audio/main         │
                    └───────────┬─────────────┘
                                │ AudioData
                    ┌───────────┴─────────────┐
                    ▼                         ▼
        ┌───────────────────┐     ┌───────────────────┐
        │   KWS Node        │     │   ASR Node        │
        │                   │     │                   │
        │ Непрерывное       │     │ По запросу        │
        │ обнаружение       │     │ (set_mode → 1/2)  │
        │ wake phrases +    │     │                   │
        │ fast commands     │     │ Grammar / Free    │
        │                   │     │                   │
        │ pub: wake_phrases │     │ pub: commands     │
        │ pub: fast_commands│     │                   │
        └───────────────────┘     └───────────────────┘
```

---

## Нода 1: KWS (Keyword Spotting) — `kws.py`

### Описание

Непрерывно слушает аудиопоток и ищет в нём заданные **ключевые фразы** (wake phrases) и **быстрые команды** (fast commands). Использует Vosk с жёсткой грамматикой (только заданные фразы), что обеспечивает высокую точность при ограниченном словаре.

### Режимы работы

| Режим | Код | Описание |
|-------|-----|----------|
| Off | `0` | Распознавание отключено, аудио игнорируется |
| On | `1` | Активное обнаружение ключевых фраз |

### Топики

#### Публикуемые

| Топик | Тип | По умолчанию | Описание |
|-------|-----|-------------|----------|
| `<namespace>/wake_phrases` | `std_msgs/msg/String` | `wake_phrases` | Обнаруженная ключевая фраза |
| `<namespace>/fast_commands` | `std_msgs/msg/String` | `fast_commands` | Обнаруженная быстрая команда |

#### Подписки

| Топик | Тип | По умолчанию | Описание |
|-------|-----|-------------|----------|
| `/respeaker_driver/audio/main` | `robohead_interfaces/msg/AudioData` | настраивается | Входной аудиопоток |

### Сервисы

#### `<namespace>/set_mode`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |

| Запрос `data` | Описание |
|----------------|----------|
| `0` | Выключить распознавание |
| `1` | Включить распознавание |

| Ответ `data` | Описание |
|---------------|----------|
| `0` или `1` | Установленный режим |
| текущий режим | Если запрошен недопустимый режим |

### Параметры конфигурации

Файл: `config/speech_recognizer_kws.yaml`

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `default_mode` | `int` | `0` | Начальный режим (0 — off, 1 — on) |
| `sample_rate` | `int` | `16000` | Частота дискретизации (должна совпадать с `respeaker_driver`) |
| `model_path` | `string` | `"vosk-model-small-ru-0.22"` | Имя каталога модели (относительно `speech_recognizer/speech_recognizer/model/`) |
| `wake_phrases` | `string[]` | `["слушай робот"]` | Список ключевых фраз для обнаружения |
| `fast_commands` | `string[]` | `["громче", "тише", "стоп"]` | Список быстрых команд |
| `ros.service_name.set_mode` | `string` | `"set_mode"` | Имя сервиса управления режимом |
| `ros.topic_name.wake_phrases` | `string` | `"wake_phrases"` | Имя топика ключевых фраз |
| `ros.topic_name.fast_commands` | `string` | `"fast_commands"` | Имя топика быстрых команд |
| `ros.topic_name.audio_input` | `string` | `"/respeaker_driver/audio/main"` | Входной аудиотопик |

### Логика работы

1. При поступлении аудиоданных передаёт их в Vosk-распознаватель с грамматикой (`wake_phrases` + `fast_commands`).
2. Когда Vosk формирует финальный результат (`AcceptWaveform` → `true`), текст проверяется:
   - Если совпадает с одной из `wake_phrases` → публикуется в `wake_phrases`.
   - Если совпадает с одной из `fast_commands` → публикуется в `fast_commands`.
3. Режим **не переключается** автоматически — KWS остаётся активным до явного отключения через сервис.

---

## Нода 2: ASR (Automatic Speech Recognition) — `asr.py`

### Описание

Распознаёт команды или произвольную речь **по запросу**. Предназначен для сценария: KWS обнаружил wake phrase → внешняя логика включает ASR → ASR распознаёт одну команду → автоматически выключается.

### Режимы работы

| Режим | Код | Описание |
|-------|-----|----------|
| Off | `0` | Распознавание отключено |
| Grammar | `1` | Распознавание из заданного списка команд. После распознавания (или таймаута) автоматически переключается в режим `0` |
| Free | `2` | Свободное распознавание (полный словарь модели). Остаётся активным до явного отключения |

### Топики

#### Публикуемые

| Топик | Тип | По умолчанию | Описание |
|-------|-----|-------------|----------|
| `<namespace>/commands` | `std_msgs/msg/String` | `commands` | Распознанная команда или текст |

**Специальные значения:**

| Значение | Описание |
|----------|----------|
| `__TIMEOUT__` | Опубликовано, если в режиме Grammar не было распознано ни одной команды в течение `t_start_max` секунд (настраивается параметром `timeout_text`) |

#### Подписки

| Топик | Тип | По умолчанию | Описание |
|-------|-----|-------------|----------|
| `/respeaker_driver/audio/main` | `robohead_interfaces/msg/AudioData` | настраивается | Входной аудиопоток |

### Сервисы

#### `<namespace>/set_mode`

| Поле | Значение |
|------|----------|
| **Тип** | `robohead_interfaces/srv/SimpleCommand` |

| Запрос `data` | Описание |
|----------------|----------|
| `0` | Выключить распознавание |
| `1` | Включить режим Grammar (распознавание из списка команд) |
| `2` | Включить режим Free (свободное распознавание) |

| Ответ `data` | Описание |
|---------------|----------|
| `0`, `1` или `2` | Установленный режим |
| текущий режим | Если запрошен недопустимый режим |

### Параметры конфигурации

Файл: `config/speech_recognizer_asr.yaml`

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `default_mode` | `int` | `0` | Начальный режим (0/1/2) |
| `sample_rate` | `int` | `16000` | Частота дискретизации |
| `model_path` | `string` | `"vosk-model-small-ru-0.22"` | Имя каталога модели |
| `timeout_text` | `string` | `"__TIMEOUT__"` | Текст, публикуемый при таймауте в режиме Grammar |
| `t_start_max` | `double` | `5.0` | Максимальное время ожидания начала речи (сек). Если за это время речь не обнаружена — таймаут |
| `t_end` | `double` | `0.5` | Время тишины после последнего слова для завершения фразы (сек) |
| `t_max` | `double` | `6.0` | Максимальная длительность одной фразы (сек) |
| `commands` | `string[]` | `["покажи уши", "поздоровайся"]` | Список распознаваемых команд (для режима Grammar) |
| `ros.service_name.set_mode` | `string` | `"set_mode"` | Имя сервиса |
| `ros.topic_name.commands` | `string` | `"commands"` | Имя топика результатов |
| `ros.topic_name.audio_input` | `string` | `"/respeaker_driver/audio/main"` | Входной аудиотопик |

### Логика работы — режим Grammar (mode=1)

1. Внешняя логика вызывает `set_mode(1)`.
2. ASR начинает передавать аудио в Vosk с грамматикой из `commands`.
3. Vosk использует **эндпоинтер** с настройками `t_start_max`, `t_end`, `t_max`:
   - Если речь не начинается за `t_start_max` секунд → таймаут.
   - Если после последнего слова тишина `t_end` секунд → фраза завершена.
   - Если фраза длится более `t_max` секунд → принудительное завершение.
4. Результат проверяется в `final_result` и `partial_result`:
   - Найденная команда → публикуется в `commands`.
   - Ничего не найдено → публикуется `__TIMEOUT__`.
5. Режим **автоматически переключается в `0` (Off)**.

### Логика работы — режим Free (mode=2)

1. Внешняя логика вызывает `set_mode(2)`.
2. ASR распознаёт произвольную речь (полный словарь модели).
3. Каждая распознанная фраза публикуется в `commands`.
4. Режим **не переключается** автоматически — остаётся активным.

---

## Типовой сценарий использования

```
1. KWS работает в режиме 1 (непрерывно слушает)
2. Пользователь говорит "слушай робот"
3. KWS публикует "слушай робот" в wake_phrases
4. Внешняя логика получает wake phrase → вызывает ASR set_mode(1)
5. ASR начинает распознавание команды
6. Пользователь говорит "покажи уши"
7. ASR публикует "покажи уши" в commands → автоматически переключается в mode=0
8. Внешняя логика выполняет команду
9. KWS продолжает слушать wake phrases
```

---

## Сборка

```bash
colcon build --symlink-install --packages-select robohead_interface speech_recognizer
```

---

## Launch-файлы

> ВАЖНО: перед запуском распознавания убедитесь, что запущен пакет respeaker_driver, который публикует аудио-данные с микрофона - иначе распознавать будет нечего.

### Запуск только KWS

```bash
ros2 launch speech_recognizer kws.launch.py
```

Namespace: `/kws/`

### Запуск только ASR

```bash
ros2 launch speech_recognizer asr.launch.py
```

Namespace: `/asr/`

### Запуск обеих нод

```bash
ros2 launch speech_recognizer speech_recognizer.launch.py
```

Namespace: `/speech_recognizer/kws/` и `/speech_recognizer/asr/`

---

## Примеры взаимодействия

### Из командной строки

```bash
ros2 launch speech_recognizer speech_recognizer.launch.py
```

```bash
# Включить KWS
ros2 service call /speech_recognizer/kws/set_mode robohead_interfaces/srv/SimpleCommand "{data: 1}"

# Включить ASR в режиме Grammar
ros2 service call /speech_recognizer/asr/set_mode robohead_interfaces/srv/SimpleCommand "{data: 1}"

# Включить ASR в режиме Free
ros2 service call /speech_recognizer/asr/set_mode robohead_interfaces/srv/SimpleCommand "{data: 2}"

# Выключить ASR
ros2 service call /speech_recognizer/asr/set_mode robohead_interfaces/srv/SimpleCommand "{data: 0}"

# Слушать wake phrases
ros2 topic echo /speech_recognizer/kws/wake_phrases

# Слушать быстрые команды
ros2 topic echo /speech_recognizer/kws/fast_commands

# Слушать распознанные команды ASR
ros2 topic echo /speech_recognizer/asr/commands
```

### Из другой ROS2 ноды (Python)

#TODO

### Из другой ROS2 ноды (C++)

#TODO

---

## Модели Vosk

Модели располагаются в каталоге `speech_recognizer/speech_recognizer/model/`. Путь задаётся параметром `model_path` (имя каталога).

| Модель | Размер | Описание |
|--------|--------|----------|
| `vosk-model-small-ru-0.22` | ~45 МБ | Компактная русская модель (рекомендуется) |
| `vosk-model-small-ru-0.21` | ~45 МБ | Предыдущая версия |
| `vosk-model-ru-0.42` | ~1.8 ГБ | Полная русская модель (высокая точность), но чрезмерно тяжёлая для Raspberry Pi 5 |

Скачать модели: [https://alphacephei.com/vosk/models](https://alphacephei.com/vosk/models)

---