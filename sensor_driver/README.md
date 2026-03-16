

# Документация: ROS2 нода `sensor_driver`

## Общее описание

Нода `sensor_driver` считывает напряжение и ток с датчика **INA219** (монитор питания), подключённого по шине **I2C**, и публикует данные о состоянии батареи в стандартном формате `sensor_msgs/msg/BatteryState`. Датчик сконфигурирован на диапазон **16 В / 5 А** с 12-битным АЦП (усреднение по 32 выборкам).

---

## Топики

### Публикуемые

### `<namespace>/battery`

| Поле | Значение |
|------|----------|
| **Тип** | `sensor_msgs/msg/BatteryState` |
| **Имя по умолчанию** | `battery` (настраивается параметром `topic_name`) |
| **Частота** | задаётся параметром `publish_rate` (по умолчанию 5 Гц) |
| **QoS** | глубина очереди 10 |

#### Заполняемые поля сообщения

| Поле | Тип | Описание |
|------|-----|----------|
| `header.stamp` | `Time` | Временная метка момента считывания |
| `voltage` | `float32` | Напряжение на шине (В) |
| `current` | `float32` | Ток (А). Положительный — зарядка, отрицательный — разрядка |
| `power_supply_status` | `uint8` | Статус питания (см. таблицу ниже) |
| `power_supply_technology` | `uint8` | `POWER_SUPPLY_TECHNOLOGY_LION` (Li-Ion) |
| `power_supply_health` | `uint8` | `POWER_SUPPLY_HEALTH_UNKNOWN` |
| `present` | `bool` | Всегда `true` |
| `location` | `string` | `"main_battery"` |
| `charge` | `float32` | `NaN` (не определяется) |
| `capacity` | `float32` | `NaN` (не определяется) |
| `design_capacity` | `float32` | `NaN` (не определяется) |
| `percentage` | `float32` | `NaN` (не определяется) |

#### Логика определения `power_supply_status`

| Условие | Статус |
|---------|--------|
| `current > charge_threshold` | `POWER_SUPPLY_STATUS_CHARGING` |
| `current < -charge_threshold` | `POWER_SUPPLY_STATUS_DISCHARGING` |
| `-charge_threshold ≤ current ≤ charge_threshold` | `POWER_SUPPLY_STATUS_NOT_CHARGING` |

---

## Параметры конфигурации

Файл: `config/sensor_driver.yaml`

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|-------------|----------|
| `topic_name` | `string` | `"battery"` | Имя топика для публикации данных о батарее |
| `publish_rate` | `int` | `5` | Частота публикации (Гц) |
| `i2c_address` | `int` | `67` (0x43) | I2C-адрес INA219 в **десятичном** формате |
| `i2c_bus` | `int` | `1` | Номер шины I2C (соответствует `/dev/i2c-<N>`) |
| `charge_threshold` | `float` | `0.1` | Порог тока (А) для определения статуса зарядки/разрядки |

---

## Конфигурация INA219

Датчик инициализируется со следующими параметрами:

| Параметр | Значение |
|----------|----------|
| Диапазон напряжения | 16 В |
| Усиление шунта | ×2 (80 мВ) |
| Разрешение АЦП | 12 бит, усреднение по 32 выборкам |
| Режим | Непрерывное измерение напряжения шунта и шины |
| Значение калибровки | 26868 |
| LSB тока | 0.1524 мА |

---

## Примеры взаимодействия

## Сборка

```bash
colcon build --symlink-install --packages-select sensor_driver
```

## Запуск

```bash
ros2 launch sensor_driver sensor_driver.launch.py 
```
Вывод при успешном запуске:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [sensor_driver_node-1]: process started with pid [14241]
[sensor_driver_node-1] [INFO] [1773674776.381881189] [sensor_driver.sensor_driver]: INITED
```

### Из командной строки

```bash
# Просмотр данных о батарее в реальном времени
ros2 topic echo /sensor_driver/battery

# Однократное чтение
ros2 topic echo /sensor_driver/battery --once

# Проверка частоты публикации
ros2 topic hz /sensor_driver/battery
```

### Из другой ROS2 ноды (C++)

#TODO

### Из другой ROS2 ноды (Python)

#TODO