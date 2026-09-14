# Автономный полёт дрона с конечным автоматом на C++/ROS2 (учебный проект)

Контроллер дрона ArduPilot через ROS2 с использованием конечного автомата в однопоточном режиме (Single Thread Executor).

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## 🚀 Возможности

- **Телеметрия**: подписка на топики `/ap/geopose/filtered` и `/ap/battery`, получение координат в градусах и высоты в метрах над уровнем моря, уровня заряда батареи. Логирование информации: время с начала полёта, координаты местоположения дрона, заряд аккумулятора.
- **Управление**: через вызовы сервисов ArduPilot и публикацию сообщений:
  - Переключение в режим `GUIDED`, сервис `ap/mode_switch`, номер режима 4
  - Армирование (`ARM`) сервис `ap/arm_motors`
  - Взлёт (`TAKEOFF`), сервис `ap/experimental/takeoff`
  - Полёт по маршруту, сообщения `geographic_msgs::msg::GeoPoseStamped`, топик `/ap/cmd_gps_pose`
  - Переключение в режим посадки (`LAND`), сервис `ap/mode_switch`, номер режима 9.
- **Асинхронность**: все вызовы сервисов выполняются асинхронно, без блокировки основного потока.
- **Тестирование**: работает в симуляторе ArduPilot SITL.
- **Обработка внештатных ситуаций**: батарея, геозона, таймаут связи.
  - При снижении уровня заряда батареи до 20% переводим дрон в режим возврата на базу (`RTL`), сервис `ap/mode_switch`, номер режима 6.
  - При снижении уровня заряда батареи до 5% - в режим посадки (`LAND`), сервис `ap/mode_switch`, номер режима 9.
  - При удалении от точки взлёта на заданное расстояние переключаемся в режим возвращения (`RTL`), сервис `ap/mode_switch`, номер режима 6.
  - При отсутствии связи более заданного интервала времени (5 сек) логируем это состояние и переходим в режим ожидания. При восстановлении связи продолжаем выполнение полёта по маршруту.
- **Задание маршрута в файле**: все точки облёта задаются в файле `src/drone_controller_cpp/src/route.csv`. Координаты широты и долготы в градусах, высота в метрах от точки старта.
- **Логирование**: запись телеметрии в `treck.csv` во время полёта (секунда, координаты, высота, заряд, режим, шаг автомата).

---

## 🧠 Архитектура

### Конечный автомат (`State Machine`)

```text
    INIT,
    SET_GUIDED_MODE,
    WAIT_GUIDED_RESPONSE,
    SEND_ARMING,
    WAIT_ARMING_RESPONSE,
    SEND_TAKEOFF,
    WAIT_TAKEOFF_COMPLETE,
    FLY_TO_POINT,
    RETURN_TO_BASE,
    WAIT_RTL_RESPONSE,
    SEND_LANDING,
    WAIT_LANDING_RESPONSE,
    MISSION_COMPLETE,
    ERROR
```

**Каждый шаг:**

- Отправляет асинхронный запрос к сервисам ArduPilot или публикует сообщение.
- В колбэке обрабатывает ответ и переключает состояние.
- Таймер (500 мс) управляет переходами, не блокируя поток.

### Компоненты

| Компонент | Назначение |
|-----------|------------|
| Подписка | Обновляет положение дрона, заряд батареи, режим. Колбэк короткий и неблокирующий. |
| Таймер | Запускает `control_loop()` каждые 500 мс. Управляет миссией по шагам. |
| Клиенты ArduPilot | `mode_switch`, `arm_motors`, `takeoff` — асинхронные вызовы с колбэками. |

## 🛠 Стек

- **OS**: `Ubuntu 22.04 LTS`
- **ROS 2**: `Humble Hawksbill`
- **DDS + MicroXRCEAgent**: `ros-humble-mavros`
- **Симулятор**: `ArduPilot SITL`
- **Язык**: `C++17`
- **Сборка** (`CMake + colcon`):
```bash
cd ~/ardu_ws
colcon build --packages-select drone_controller_cpp
source install/setup.bash
```

## 📁 Структура пакета
```text
drone_controller_cpp/
├── src/
│   └── controller_node_dds.cpp   # Реализация узла
├── CMakeLists.txt
├── package.xml
└── README.md
```
## 🚀 Запуск

### 1️⃣ Симулятор SITL (терминал 1)
```bash
cd ~/ardu_ws/src/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --enable-DDS
```
### 2️⃣ MicroXRCEAgent (терминал 2)
```bash
MicroXRCEAgent udp4 --port 2019
```
### 3️⃣ Узел управления (терминал 3)
```bash
cd ~/ardu_ws
source install/setup.bash
ros2 run drone_controller_cpp dds_machine
```
### Файлы проекта
- `route.csv` — маршрут (смещения от home в градусах и метрах).
- `treck.csv` — журнал полёта (создаётся автоматически при запуске узла).

## 📊 Пример вывода
```text
[INFO] [1789370558.776679445] [minimal_controller]: Контроллер запущен в один поток.
[INFO] [1789370559.276890130] [minimal_controller]: 0,-35.3632622,149.1652374,584.09,0.0,STABILIZE,INIT
[INFO] [1789370559.776860769] [minimal_controller]: Запрос режима GUIDED (4)...
[INFO] [1789370560.276897615] [minimal_controller]: Запрос на ARM...
[INFO] [1789370560.296591302] [minimal_controller]: Arm response: result=1
[INFO] [1789370560.776865447] [minimal_controller]: Запрос взлета на 5 метров...
[INFO] [1789370560.787539686] [minimal_controller]: Takeoff service response received: status=1
[INFO] [1789370563.776937875] [minimal_controller]: 5,-35.3632622,149.1652374,584.09,99.0,GUIDED,WAIT_TAKEOFF_COMPLETE
[INFO] [1789370567.776987114] [minimal_controller]: 9,-35.3632622,149.1652374,589.10,98.0,GUIDED,FLY_TO_POINT
[INFO] [1789370567.777134416] [minimal_controller]: Целевая точка достигнута...
[INFO] [1789370571.777050376] [minimal_controller]: 13,-35.3631210,149.1652374,592.15,97.0,GUIDED,FLY_TO_POINT
[INFO] [1789370575.777154176] [minimal_controller]: 17,-35.3630295,149.1652374,593.08,97.0,GUIDED,FLY_TO_POINT
[ERROR] [1789370578.277129357] [minimal_controller]: Потеря связи: 5.4 секунд
[ERROR] [1789370583.277253390] [minimal_controller]: Потеря связи: 10.4 секунд
[INFO] [1789370584.777252590] [minimal_controller]: Связь восстановлена...
[INFO] [1789370584.777357344] [minimal_controller]: 26,-35.3623581,149.1652374,599.07,93.0,GUIDED,FLY_TO_POINT
[INFO] [1789370584.777383142] [minimal_controller]: Целевая точка достигнута...
[INFO] [1789370588.777368991] [minimal_controller]: 30,-35.3623619,149.1654205,602.66,92.0,GUIDED,FLY_TO_POINT
[INFO] [1789370592.777513128] [minimal_controller]: 34,-35.3623619,149.1656952,605.21,91.0,GUIDED,FLY_TO_POINT
[ERROR] [1789370596.777496399] [minimal_controller]: Потеря связи: 5.4 секунд
[INFO] [1789370599.277503975] [minimal_controller]: Связь восстановлена...
[INFO] [1789370599.277653883] [minimal_controller]: 40,-35.3623619,149.1661530,609.07,89.0,GUIDED,FLY_TO_POINT
[INFO] [1789370599.277692245] [minimal_controller]: Целевая точка достигнута...
[INFO] [1789370603.777597444] [minimal_controller]: 45,-35.3625488,149.1661530,605.62,87.0,GUIDED,FLY_TO_POINT
[INFO] [1789370607.777677476] [minimal_controller]: 49,-35.3628311,149.1661377,602.92,86.0,GUIDED,FLY_TO_POINT
[INFO] [1789370611.777764078] [minimal_controller]: 53,-35.3628311,149.1661377,602.92,86.0,GUIDED,FLY_TO_POINT
[ERROR] [1789370612.277761015] [minimal_controller]: Потеря связи: 5.2 секунд
[INFO] [1789370614.277817695] [minimal_controller]: Связь восстановлена...
[INFO] [1789370614.278014476] [minimal_controller]: Целевая точка достигнута...
[INFO] [1789370615.777799916] [minimal_controller]: 57,-35.3632660,149.1661224,598.20,84.0,GUIDED,FLY_TO_POINT
[INFO] [1789370619.777917001] [minimal_controller]: 61,-35.3632660,149.1658325,594.08,83.0,GUIDED,FLY_TO_POINT
[INFO] [1789370623.777975299] [minimal_controller]: 65,-35.3632622,149.1654053,590.39,82.0,GUIDED,FLY_TO_POINT
[INFO] [1789370626.277989938] [minimal_controller]: Целевая точка достигнута...
[WARN] [1789370626.278035780] [minimal_controller]: Нет больше waypoints
[INFO] [1789370626.778039429] [minimal_controller]: Запрос режима LAND (9)...
[INFO] [1789370627.778038712] [minimal_controller]: 69,-35.3632622,149.1652222,588.84,81.0,LAND,WAIT_LANDING_RESPONSE
[INFO] [1789370631.778149229] [minimal_controller]: 73,-35.3632660,149.1652374,586.79,79.0,LAND,WAIT_LANDING_RESPONSE
[INFO] [1789370635.778190353] [minimal_controller]: 77,-35.3632622,149.1652374,584.78,78.0,LAND,WAIT_LANDING_RESPONSE
[INFO] [1789370636.778323877] [minimal_controller]: Миссия завершена успешно!
```
## 🧪 Пример использования

### После запуска всех компонентов узел:

- Подписывается на топики `/ap/geopose/filtered` и `/ap/battery` (телеметрия).
- Автоматически отправляет команды на взлёт, облёт по квадрату и посадку.
- Отслеживает состояние заряда батареи, удаление от базы, связь с узлом управления.
- Выводит информацию о ходе полёта.

При успешном запуске вы увидите в консоли сообщения о взлёте, местоположении и посадке, а также перемещение дрона на карте.

### Обработка ошибок
- При снижении уровня заряда батареи до 20% выводится сообщение о возврате на базу, при снижении уровня заряда до 5% о переходе в режим посадки.
- При удалении от точки взлёта на заданное расстояние также переключаемся в режим возвращения.
- При отсутствии связи более 5 сек логируем это состояние и переходим в режим ожидания. При восстановлении связи продолжаем выполнение полёта по маршруту.

## 📌 Планы по развитию

- [x] Полёт по точкам (waypoints)

- [x] Посадка (LAND)

- [x] Обработка ошибок (таймауты, аварийная посадка при ошибке)

- [x] Обработка аварий (заряд аккумулятора, вылет за периметр)

- [ ] Логирование телеметрии в CSV

- [x] Задание маршрутной карты в файле route.csv

## [История изменений](CHANGELOG.md)

## 📚 Источники

[ArduPilot SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

[ROS 2 Humble](https://docs.ros.org/en/humble/)


## 📬 Контакты

Автор: [ituretsky]  
GitHub: [ituretsky](https://github.com/ituretsky)
