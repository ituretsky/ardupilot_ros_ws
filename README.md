# ROS2 Workspace для управления ArduPilot

Этот репозиторий содержит ROS2 (Humble) воркспейс для управления симулятором ArduPilot SITL. Реализован гибридный подход:
- **Телеметрия** получается через DDS-мост (MicroXRCEAgent + топики `/ap/...`).
- **Команды управления** (взлёт, движение) отправляются через DroneKit, подключённый к MAVProxy.

Такой подход позволяет использовать современный DDS для данных и надёжный MAVLink-канал для команд.

---

## 📂 Структура репозитория

ardupilot_ros_ws/
├── src/
│ └── drone_controller/ # ROS-пакет для управления
│ ├── drone_controller/
│ │ ├── init.py
│ │ └── controller_node.py # Главный узел (гибридный)
│ ├── package.xml
│ └── setup.py
├── README.md
└── .gitignore

---

## 🛠️ Требования

- Ubuntu 22.04
- ROS2 Humble
- ArduPilot SITL (собран с поддержкой DDS)
- Python 3.10+
- Пакеты: `dronekit`, `pymavlink`, `microxrcedds_agent`

---

## 🚀 Сборка и запуск

### 1. Клонируйте репозиторий и соберите воркспейс

cd ~
git clone https://github.com/ituretsky/ardupilot_ros_ws.git
cd ardupilot_ros_ws
colcon build --packages-select drone_controller
source install/setup.bash

### 2. Запустите MicroXRCEAgent

MicroXRCEAgent udp4 --port 2019

### 3. Запустите ArduPilot SITL с поддержкой DDS

cd ~/ardu_ws/src/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --enable-DDS

### 4. Запустите ROS-узел

source ~/ardupilot_ros_ws/install/setup.bash
ros2 run drone_controller controller_node

🧪 Пример использования

После запуска всех компонентов узел:

    Подписывается на топик /ap/pose/filtered (телеметрия).

    Выводит позицию дрона.

    Автоматически отправляет команду взлёта на 5 метров (через DroneKit).

При успешном запуске вы увидите сообщения о взлёте и рост высоты в телеметрии.

📌 Примечания

    Параметр DDS_ENABLE в ArduPilot может отсутствовать, это нормально — DDS-модуль активируется флагом --enable-DDS при запуске.

    Для изменения логики управления редактируйте controller_node.py.

    Для работы без DroneKit можно использовать MAVROS (экспериментально).
    

