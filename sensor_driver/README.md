
# Документация: ROS2 пакет `sensor_driver`

**Назначение:** Пакет `sensor_driver` обеспечивает взаимодействие с датчиком тока и напряжения **INA219** (монитор питания) через шину **I2C** и публикует данные о состоянии батареи в стандартном формате `sensor_msgs/msg/BatteryState`.

---

## 📁 Структура пакета

```text
sensor_driver/
├── CMakeLists.txt                 # Правила сборки (ament_cmake)
├── package.xml                    # Зависимости и метаданные пакета
├── README.md                      # Краткое описание
├── src/
│   └── main.cpp                   # Исходный код ноды (C++17)
├── config/
│   └── sensor_driver.yaml         # Параметры конфигурации
└── launch/
    └── sensor_driver.launch.py    # Launch-файл для запуска
```

---

## 📡 Топики

### Публикуемые

#### `<namespace>/battery`

| Параметр | Значение |
|----------|----------|
| **Тип сообщения** | `sensor_msgs/msg/BatteryState` |
| **Имя по умолчанию** | `battery` (настраивается параметром `topic_name`) |
| **Частота публикации** | задаётся параметром `publish_rate` (по умолчанию **5 Гц**) |
| **QoS** | глубина очереди **10** |
| **Пространство имён** | задаётся в launch-файле (по умолчанию: `sensor_driver`) |

#### Заполняемые поля сообщения

| Поле | Тип | Описание |
|------|-----|----------|
| `header.stamp` | `Time` | Временная метка момента считывания данных |
| `voltage` | `float32` | Напряжение на шине, **В** |
| `current` | `float32` | Ток через батарею, **А** (*> 0 — зарядка, < 0 — разрядка*) |
| `power_supply_status` | `uint8` | Статус питания (см. таблицу ниже) |
| `power_supply_technology` | `uint8` | `POWER_SUPPLY_TECHNOLOGY_LION` (Li-Ion) |
| `power_supply_health` | `uint8` | `POWER_SUPPLY_HEALTH_UNKNOWN` |
| `present` | `bool` | Всегда `true` (датчик обнаружен) |
| `location` | `string` | `"main_battery"` |
| `charge`, `capacity`, `design_capacity`, `percentage` | `float32` | `NaN` (не определяются данным датчиком) |
| `cell_voltage` | `float32[]` | Пустой массив |
| `serial_number` | `string` | Пустая строка |

#### Логика определения `power_supply_status`

| Условие | Значение | Константа ROS2 |
|---------|----------|---------------|
| `current > charge_threshold` | Зарядка | `POWER_SUPPLY_STATUS_CHARGING` |
| `current < -charge_threshold` | Разрядка | `POWER_SUPPLY_STATUS_DISCHARGING` |
| `-charge_threshold ≤ current ≤ charge_threshold` | Не заряжается | `POWER_SUPPLY_STATUS_NOT_CHARGING` |

> 💡 Порог `charge_threshold` (по умолчанию **0.1 А**) необходим для фильтрации шумов измерения.

---

## ⚙️ Параметры конфигурации

Файл: `config/sensor_driver.yaml`

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `topic_name` | `string` | `"battery"` | Имя топика (без префикса пространства имён) |
| `publish_rate` | `int` | `5` | Частота публикации данных |
| `i2c_address` | `int` | `67` | I2C-адрес INA219 **в десятичном формате** |
| `i2c_bus` | `int` | `1` | Номер I2C-шины (соответствует `/dev/i2c-<N>`) |
| `charge_threshold` | `float` | `0.1` | Порог тока (А) для определения статуса зарядки |

> ⚠️ **Важно:** Параметр `i2c_address` указывается в **десятичной** системе счисления.  
> Пример: адрес `0x43` → записывайте как `67`.

---

## 🔧 Конфигурация INA219

Датчик инициализируется со следующими аппаратными настройками:

| Параметр | Значение | Описание |
|----------|----------|----------|
| Диапазон напряжения | 16 В | `BusVoltageRange::RANGE_16V` |
| Усиление шунта | ×2 (±80 мВ) | `Gain::DIV_2_80MV` |
| Разрешение АЦП | 12 бит, 32 выборки | `ADCResolution::ADCRES_12BIT_32S` |
| Режим работы | Непрерывный | `Mode::SANDBVOLT_CONTINUOUS` |
| Значение калибровки | 26868 | Регистр `REG_CALIBRATION` |
| LSB тока | 0.1524 мА | Шаг квантования тока |

> 📐 Расчёт тока: `I [мА] = raw_value × 0.1524`  
> 📐 Расчёт напряжения: `U [В] = (raw_value >> 3) × 0.004`

---

## 🛠️ Сборка и запуск

### Сборка пакета

```bash
# В корневой директории workspace
colcon build --symlink-install --packages-select sensor_driver
source install/setup.bash
```

> 💡 Флаг `--symlink-install` позволяет редактировать файлы конфигурации и launch-файлы без повторной сборки.

### Запуск через launch-файл

```bash
ros2 launch sensor_driver sensor_driver.launch.py
```

**Ожидаемый вывод при успешном запуске:**
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [sensor_driver_node-1]: process started with pid [4608]
[sensor_driver_node-1] [INFO] [1776683390.095654952] [sensor_driver.sensor_driver]: INITED
```

### Проверка работы

```bash
# Просмотр данных в реальном времени
ros2 topic echo /sensor_driver/battery

# Однократное чтение
ros2 topic echo /sensor_driver/battery --once

# Проверка частоты публикации
ros2 topic hz /sensor_driver/battery

# Просмотр структуры сообщения
ros2 interface show sensor_msgs/msg/BatteryState
```

---

## 🐛 Режим отладки

По-умолчанию пакет `sensor_driver` запускается автоматически вместе со всеми остальными зависимостями пакета `robohead_controller` (внутри namespace `/robohead/...`) за счёт системного сервиса `robohead.service`. Для отладки рекомендуется остановить сервис (и соответственно все зависимости) и вручную запустить только `sensor_driver`.

### Шаг 1. Остановка фоновых сервисов

```bash
sudo systemctl stop robohead.service
```
> После перезагрузки системы сервис снова запустится автоматически. Или, для ручного возобновления работы без перезагрузки используйте `sudo systemctl start robohead.service`

### Шаг 2. Запуск ноды вручную

```bash
# В терминале 1: запуск драйвера
ros2 launch sensor_driver sensor_driver.launch.py

# В терминале 2: просмотр данных
ros2 topic echo /sensor_driver/battery
```

### Особенности режима отладки

| Аспект | `robohead_controller` | Отладка |
|--------|----------|---------|
| **Пространство имён** | Задаётся пакетом `robohead_controller`: `/robohead/sensor_driver/battery` | Фиксировано: `/sensor_driver/battery` |
| **Конфигурация** | Используется `robohead_controller/config/sensor_driver.yaml` | Используется `sensor_driver/config/sensor_driver.yaml` |

---

## ❗ Возможные ошибки и решения

| Ошибка | Возможная причина | Решение |
|--------|------------------|---------|
| `Failed to open I2C device` | Нет прав доступа к `/dev/i2c-*` | `sudo usermod -aG i2c $USER` + перелогин |
| `Failed to set I2C slave address` | Неверный адрес или устройство не отвечает | Проверьте `i2cdetect -y 1`, убедитесь, что адрес `0x43` отображается |
| `I2C read/write failed` | Проблема с подключением или помехи на шине | Проверьте пайку, подтягивающие резисторы (4.7 кОм), длину проводов |
| Нода не публикует данные | Ошибка инициализации, нода упала | Проверьте `ros2 node list` и `journalctl` / вывод в консоль |
| Неверные значения тока | Неверная калибровка или параметры INA219 | Убедитесь, что `setCalibration_16V_5A()` соответствует вашей схеме |

### Диагностика I2C

```bash
# Поиск устройств на шине 1
i2cdetect -y 1

# Чтение регистра (пример: конфиг)
i2cget -y 1 0x43 0x00 w

# Тестовая запись (осторожно!)
i2cset -y 1 0x43 0x00 0x3995 w
```

---
