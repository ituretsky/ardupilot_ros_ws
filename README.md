# Автономный полёт дрона с конечным автоматом на C++/ROS2 (учебный проект)

Контроллер дрона ArduPilot через ROS2 с использованием конечного автомата в однопоточном режиме (Single Thread Executor).

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## 🚀 Возможности

- **Телеметрия**: подписка на топик `/ap/geopose/filtered`, получение координат в градусах и высоты в метрах над уровнем моря, вывод местоположения дрона в реальном времени.
- **Управление**: через вызовы сервисов ArduPilot и публикацию сообщений:
  - Переключение в режим `GUIDED`, сервис `ap/mode_switch`, номер режима 4
  - Армирование (`ARM`) сервис `ap/arm_motors`
  - Взлёт (`TAKEOFF`), сервис `ap/experimental/takeoff`
  - Полёт по маршруту, сообщения `geographic_msgs::msg::GeoPoseStamped`, топик `/ap/cmd_gps_pose`
  - Переключение в режим посадки (`LAND`), сервис `ap/mode_switch`, номер режима 9
- **Асинхронность**: все вызовы сервисов выполняются асинхронно, без блокировки основного потока.
- **Тестирование**: работает в симуляторе ArduPilot SITL.

---

## 🧠 Архитектура

### Конечный автомат (`State Machine`)

```text
INIT → SET_GUIDED_MODE → WAIT_GUIDED_RESPONSE → SEND_ARMING → WAIT_ARMING_RESPONSE → SEND_TAKEOFF → WAIT_TAKEOFF_COMPLETE → FLY_TO_POINT → SEND_LANDING → WAIT_LANDING_RESPONSE → MISSION_COMPLETE → ERROR
```
**Каждый шаг:**

- Отправляет асинхронный запрос к сервисам ArduPilot или публикует сообщение.
- В колбэке обрабатывает ответ и переключает состояние.
- Таймер (500 мс) управляет переходами, не блокируя поток.

### Компоненты

| Компонент | Назначение |
|-----------|------------|
| Подписка | Обновляет положение дрона. Колбэк короткий и неблокирующий. |
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
## 📊 Пример вывода
```text
[INFO] [1788590757.612152627] [minimal_controller]: Контроллер запущен в один поток.
[INFO] [1788590757.700979119] [minimal_controller]: [ТЕЛЕМЕТРИЯ] XYZ: [-35.3632622, 149.1652374, 584.0899658]
[INFO] [1788590758.612281013] [minimal_controller]: Запрос режима GUIDED (4)...
[INFO] [1788590759.112248418] [minimal_controller]: Запрос на ARM...
[INFO] [1788590759.136838787] [minimal_controller]: Arm response: result=1
[INFO] [1788590759.612294338] [minimal_controller]: Запрос взлета на 5 метров...
[INFO] [1788590759.623593305] [minimal_controller]: Takeoff service response received: status=1
[INFO] [1788590766.112408408] [minimal_controller]: Запрос полёта в следующую точку...
[INFO] [1788590777.718150966] [minimal_controller]: [ТЕЛЕМЕТРИЯ] XYZ: [-35.3624916, 149.1652374, 589.1099854]
[INFO] [1788590780.152858434] [minimal_controller]: Запрос полёта в следующую точку...
[INFO] [1788590787.733402813] [minimal_controller]: [ТЕЛЕМЕТРИЯ] XYZ: [-35.3623619, 149.1657410, 589.0800171]
[INFO] [1788590792.164063593] [minimal_controller]: Запрос полёта в следующую точку...
[INFO] [1788590797.744314109] [minimal_controller]: [ТЕЛЕМЕТРИЯ] XYZ: [-35.3625832, 149.1661530, 589.0999756]
[INFO] [1788590806.230699082] [minimal_controller]: Запрос полёта в следующую точку...
[INFO] [1788590817.812129786] [minimal_controller]: [ТЕЛЕМЕТРИЯ] XYZ: [-35.3632622, 149.1652679, 589.0700073]
[INFO] [1788590818.519420632] [minimal_controller]: Запрос полёта в следующую точку...
[INFO] [1788590818.613216734] [minimal_controller]: Запрос режима LAND (9)...
[INFO] [1788590819.113228784] [minimal_controller]: Миссия завершена успешно!
```
## 🧪 Пример использования

После запуска всех компонентов узел:

    Подписывается на топик /ap/geopose/filtered (телеметрия).

    Выводит позицию дрона.

    Автоматически отправляет команды на взлёт, облёт по квадрату и посадку.

При успешном запуске вы увидите в консоли сообщения о взлёте, местоположении и посадке, а также перемещение дрона на карте.


## 📌 Планы по развитию

- [x] Полёт по точкам (waypoints)

- [x] Посадка (LAND)

- [ ] Обработка ошибок (таймауты, аварийная посадка при ошибке)

- [ ] Обработка аварий (заряд аккумулятора, вылет за периметр)

- [ ] Логирование телеметрии в CSV


## [История изменений](CHANGELOG.md)

## 📚 Источники

[ArduPilot SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

[ROS 2 Humble](https://docs.ros.org/en/humble/)


## 📬 Контакты

Автор: [ituretsky]  
GitHub: [ituretsky](https://github.com/ituretsky)