#include <rclcpp/rclcpp.hpp> // База для ROS 2 узлов
#include <cmath>             // Математика
#include <vector>            // Вектора
#include <memory>            // Для умных указателей
// #include <future>         // На будущее
#include <string>            // Строковые переменные
#include <chrono>            // Временные библиотеки
#include <fstream>           // Работа с файлами
#include <sstream>           // Потоки
#include <iomanip>           // Манипуляторы
#include <cstdint>           // Для данных с фиксированной шириной
#include <filesystem>        // Файлы
#include <ament_index_cpp/get_package_share_directory.hpp>

// Интерфейсы ArduPilot
#include <ardupilot_msgs/srv/arm_motors.hpp>
#include <ardupilot_msgs/srv/takeoff.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>
#include <ardupilot_msgs/msg/status.hpp>
#include <ardupilot_msgs/msg/global_position.hpp>

// Интерфейсы ROS 2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using namespace std::chrono_literals; // Для записи 2s, 500ms и т.д.

class MinimalController : public rclcpp::Node {
public:

// Список сотояний конечного автомата
  enum class FlightState {
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
  };

// Структура точки маршрута: широта, долгота, высота
  struct Waypoint {
    double lat;
    double lon;
    float alt;
  };

// Файлы
  std::string route_file_;          // маршрут
  std::string log_file_;            // журнал
  std::ofstream track_file;         // поток журнала

// Состояние автомата и миссии
  FlightState current_state_;       // шаг автомата
  std::vector<Waypoint> waypoints_; // маршрутная карта
  size_t target_waypoint_idx_;      // номер точки маршрута
  float current_charge_;            // заряд батареи
  uint8_t current_mode_;            // режим ArduPilot
  bool err_flag_ = false;           // обрыв связи

// Время запуска узла и крайнего сеанса связи
  rclcpp::Time start_time_;
  rclcpp::Time latest_telemetry_time_;

// Координаты дрона
  double current_lat_{0.0};
  double current_lon_{0.0};
  double current_alt_{0.0};

// Начальное положение точки взлёта с координатами аэропорта Канберры
  double home_lat = -35.36326217651367;
  double home_lon =  149.1652374267578;
  double home_alt =  584.09;   // Высота над уровнем моря

  float alt = 15.0;            // Высота над крышей дома
  int max_distance_;           // Предельное удаление
  int low_battery_;            // Критический заряд


// ROS 2 интерфейсы
  rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr mode_client_;
  rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr arming_client_;
  rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedPtr takeoff_client_;
  
  rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bat_sub_;
  rclcpp::Subscription<ardupilot_msgs::msg::Status>::SharedPtr mode_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

// Конструктор
  MinimalController() : Node("minimal_controller"), current_state_(FlightState::INIT),
                        target_waypoint_idx_(0) 
  {
// 1. Клиенты сервисов ArduPilot
    mode_client_ = this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");
    arming_client_ = this->create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");
    takeoff_client_ = this->create_client<ardupilot_msgs::srv::Takeoff>("/ap/experimental/takeoff");

// 2. Издание и подписки
    setpoint_pub_ = this->create_publisher<ardupilot_msgs::msg::GlobalPosition>("/ap/cmd_gps_pose", 10);

    auto telemetry_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pose_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
      "/ap/geopose/filtered", telemetry_qos,
      [this](const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
        this->pose_telemetry_callback(msg);
      }
    );

    bat_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/ap/battery", telemetry_qos,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        this->batstate_telemetry_callback(msg);
      }
    );

    mode_sub_ = this->create_subscription<ardupilot_msgs::msg::Status>(
      "/ap/status", telemetry_qos,
      [this](const ardupilot_msgs::msg::Status::SharedPtr msg) {
        this->ap_status_telemetry_callback(msg);
      }
    );

// 3. Считываем маршрут из файла
    std::string pkg_share = ament_index_cpp::get_package_share_directory("drone_controller_cpp");
    route_file_ = pkg_share + "/missions/route.csv";
    std::ifstream file(route_file_);
    if(!file.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "Ошибка: не удалось открыть файл маршрута: %s", route_file_.c_str());
      throw std::runtime_error("route file not found");
    }
    std::string line;
    std::getline(file, line);            // пропускаем заголовок

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string token;
      Waypoint p;
        
      getline(ss, token, ','); p.lat = stod(token) + home_lat;
      getline(ss, token, ','); p.lon = stod(token) + home_lon;
      getline(ss, token, ','); p.alt = stod(token);
      waypoints_.push_back(p);
    }
    file.close();
    alt = waypoints_.at(0).alt;            // высота из первой строки маршрута

// 4. Запускаем таймер автомата (2 Гц) и засекаем старт
    timer_ = this->create_wall_timer(500ms, [this]() { this->control_loop(); });
    start_time_ = this->get_clock()->now();
    latest_telemetry_time_ = start_time_;  // просто инициализируем не нулевым значением
    RCLCPP_INFO(this->get_logger(), "Контроллер запущен в один поток.");

// 5. Параметры
    this->declare_parameter<int>("md", 1000);
    this->declare_parameter<int>("lb", 20);

    max_distance_ = this->get_parameter("md").as_int();
    low_battery_ = this->get_parameter("lb").as_int();

// 6. Создаём журнальный файл, пишем заголовок
    std::string log_dir = std::string(getenv("HOME")) + "/drone_logs";
    std::filesystem::create_directories(log_dir);
    log_file_ = log_dir + "/track.csv";
    track_file.open(log_file_);
    if (track_file.is_open()) {
      RCLCPP_INFO(this->get_logger(), "Журнал полёта: %s", log_file_.c_str());
      track_file << "секунда,широта,долгота,высота,заряд,режим,шаг\n";
    }
  }

// Деструктор
  ~MinimalController() {
    track_file.flush();
    track_file.close();
  }

private:
// Колбэки подписок:
  void pose_telemetry_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
    latest_telemetry_time_ = this->get_clock()->now();
    current_lat_ = msg->pose.position.latitude;
    current_lon_ = msg->pose.position.longitude;
    current_alt_ = msg->pose.position.altitude;
  }

  void batstate_telemetry_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    current_charge_ = msg->percentage * 100;
  }

  void ap_status_telemetry_callback(const ardupilot_msgs::msg::Status::SharedPtr msg) {
    current_mode_ = msg->mode;
  }

// Основной цикл
  void control_loop() {
// Проверяем наличие связи
    auto now = this->get_clock()->now();
    auto elapsed = (now - latest_telemetry_time_).seconds();
    if (elapsed > 5.0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Потеря связи: %.1f секунд", elapsed);
      err_flag_ = true;
      return;
    } 
    if (err_flag_) {
      RCLCPP_INFO(this->get_logger(), "Связь восстановлена...");
      err_flag_ = false;
    }

// Выводим телеметрию и пишем в лог
    int elapsed_sec = static_cast<int>((now - start_time_).seconds());

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(7)
        << elapsed_sec << ','
        << current_lat_ << ','
        << current_lon_ << ','
        << std::setprecision(2) << current_alt_ << ','
        << std::setprecision(1) << current_charge_ << ','
        << mode_to_string(current_mode_) << ','
        << state_to_string(current_state_);
    std::string log_line = oss.str();

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "%s", log_line.c_str());

    if (track_file.is_open()) track_file << log_line << "\n";

// Конечный автомат
    switch (current_state_) {
      case FlightState::INIT:
        current_state_ = FlightState::SET_GUIDED_MODE;
        break;
      case FlightState::SET_GUIDED_MODE:
        RCLCPP_INFO(this->get_logger(), "Запрос режима GUIDED (4)...");
        send_mode_request(4);
        current_state_ = FlightState::WAIT_GUIDED_RESPONSE;
        break;
      case FlightState::WAIT_GUIDED_RESPONSE:
        break;
      case FlightState::SEND_ARMING:
        RCLCPP_INFO(this->get_logger(), "Запрос на ARM...");
        send_arm_request(true);
        current_state_ = FlightState::WAIT_ARMING_RESPONSE;
        break;
      case FlightState::WAIT_ARMING_RESPONSE:
        break;
      case FlightState::SEND_TAKEOFF:
        RCLCPP_INFO(this->get_logger(), "Запрос взлета на %d метров...", static_cast<int>(alt));
        send_takeoff_request(alt);
        current_state_ = FlightState::WAIT_TAKEOFF_COMPLETE;
        break;
      case FlightState::WAIT_TAKEOFF_COMPLETE:
        if (std::abs(home_alt + alt - current_alt_) < 0.5)
          current_state_ = FlightState::FLY_TO_POINT;
        break;
      case FlightState::FLY_TO_POINT: {
        if (current_charge_ < low_battery_) {
          current_state_ = FlightState::RETURN_TO_BASE;    // если заряд меньше 20%
          break;
        }
        double distance_from_base = haversine(current_lat_, current_lon_, home_lat, home_lon);
        if (distance_from_base > max_distance_) {
          current_state_ = FlightState::RETURN_TO_BASE;    // если достигли максимального удаления от базы
          break;
        }
        publish_target_waypoint();       // публикация целевых координат        
        check_waypoint_arrival();        // проверка прибытия в точку
        break;
      }
      case FlightState::RETURN_TO_BASE:
        RCLCPP_INFO(this->get_logger(), "Запрос режима RTL (6)...");
        send_mode_request(6);
        current_state_ = FlightState::WAIT_RTL_RESPONSE;
        break;
      case FlightState::WAIT_RTL_RESPONSE:
        if (current_charge_ < 5) {
          current_state_ = FlightState::SEND_LANDING;        // если заряд меньше 5%
          break;
        }
        if ((current_alt_ - home_alt) < 0.5) current_state_ = FlightState::MISSION_COMPLETE;
        break;
      case FlightState::SEND_LANDING:
        RCLCPP_INFO(this->get_logger(), "Запрос режима LAND (9)...");
        send_mode_request(9);
        current_state_ = FlightState::WAIT_LANDING_RESPONSE;
        break;
      case FlightState::WAIT_LANDING_RESPONSE:
        if ((current_alt_ - home_alt) < 0.5) current_state_ = FlightState::MISSION_COMPLETE;
        break;
      case FlightState::MISSION_COMPLETE:
        RCLCPP_INFO(this->get_logger(), "Миссия завершена успешно!");
        rclcpp::shutdown();
        break;
      case FlightState::ERROR:
        RCLCPP_FATAL(this->get_logger(), "Завершение с ошибкой...");
        rclcpp::shutdown();
        break;          
    }
  }

  void send_mode_request(uint8_t mode) {
    if (!mode_client_->wait_for_service(0s)) return;
    auto request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
    request->mode = mode;
    mode_client_->async_send_request(request, [this](const rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedFuture future) {
        this->mode_response_callback(future);
    });
  }
  void mode_response_callback(rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedFuture future) {
    if (future.get()->status) {
        if (current_state_ == FlightState::WAIT_GUIDED_RESPONSE) current_state_ = FlightState::SEND_ARMING;
        else if (current_state_ == FlightState::WAIT_LANDING_RESPONSE) return;
        else if (current_state_ == FlightState::WAIT_RTL_RESPONSE) return;
    } else {
        if (current_state_ == FlightState::WAIT_GUIDED_RESPONSE) current_state_ = FlightState::SET_GUIDED_MODE;
        else if (current_state_ == FlightState::WAIT_LANDING_RESPONSE) current_state_ = FlightState::SEND_LANDING;
        else if (current_state_ == FlightState::WAIT_RTL_RESPONSE) current_state_ = FlightState::RETURN_TO_BASE;
    }
  }

  void send_arm_request(bool arm) {
    if (!arming_client_->wait_for_service(0s)) return;
    auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
    request->arm = arm;
    arming_client_->async_send_request(request, [this](const rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture future) {
      this->arm_response_callback(future);
    });
  }
  void arm_response_callback(rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Arm response: result=%d", response->result);
    if (response->result) {
      current_state_ = FlightState::SEND_TAKEOFF;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Arming failed! Retrying...");
      current_state_ = FlightState::SEND_ARMING;
    }
  }

  void send_takeoff_request(float altitude) {
    if (!takeoff_client_->wait_for_service(0s)) return;
    auto request = std::make_shared<ardupilot_msgs::srv::Takeoff::Request>();
    request->alt = altitude;
    takeoff_client_->async_send_request(request, [this](const rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedFuture future) {
      this->takeoff_response_callback(future);
    });
  }
  void takeoff_response_callback(rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedFuture future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Takeoff service response received: status=%d", response->status);
    if (response->status) {
      current_state_ = FlightState::WAIT_TAKEOFF_COMPLETE;
    } else {
      current_state_ = FlightState::SEND_TAKEOFF;
    }
  }

  void publish_target_waypoint() {
    if (target_waypoint_idx_ >= waypoints_.size()) {
      RCLCPP_WARN(this->get_logger(), "Нет больше waypoints");
      return;
    }
    auto target = waypoints_[target_waypoint_idx_];
    ardupilot_msgs::msg::GlobalPosition msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";                     // <-- Глобальный фрейм
    msg.coordinate_frame = msg.FRAME_GLOBAL_REL_ALT; // 6 — высота относительно дома
    msg.type_mask = 0;                               // 0 — все поля активны
    msg.latitude = target.lat;
    msg.longitude = target.lon;
    msg.altitude = target.alt;

    // Не хотим задавать скорость и рысканье, оставляем их нулевыми
    msg.velocity.linear.x = 0.0;
    msg.velocity.linear.y = 0.0;
    msg.velocity.linear.z = 0.0;
    msg.yaw = 0.0;

    setpoint_pub_->publish(msg);
  }

// Пересчёт градусов в расстояние
  double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;                      // радиус Земли в метрах
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180.0) * 
               cos(lat2 * M_PI / 180.0) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
  }

// Проверка прибытия в точку, точность по горизонтали 1,6м, по вертикали - 0,5м
  void check_waypoint_arrival() {
    if (target_waypoint_idx_ >= waypoints_.size()) return;
    auto target = waypoints_[target_waypoint_idx_];
    double horizontal_dist = haversine(current_lat_, current_lon_, target.lat, target.lon);
    double vertical_dist = std::abs(current_alt_ - (home_alt + target.alt));
    if (vertical_dist < 0.5 && horizontal_dist < 1.6) {
      target_waypoint_idx_++;
      RCLCPP_INFO(this->get_logger(), "Целевая точка достигнута...");
      if (target_waypoint_idx_ >= waypoints_.size()) current_state_ = FlightState::SEND_LANDING;
    }
  }

  std::string state_to_string(FlightState s) {
    switch (s) {
      case FlightState::INIT:                 return "INIT";
      case FlightState::SET_GUIDED_MODE:      return "SET_GUIDED_MODE";
      case FlightState::WAIT_GUIDED_RESPONSE: return "WAIT_GUIDED_RESPONSE";
      case FlightState::SEND_ARMING:          return "SEND_ARMING";
      case FlightState::WAIT_ARMING_RESPONSE: return "WAIT_ARMING_RESPONSE";
      case FlightState::SEND_TAKEOFF:         return "SEND_TAKEOFF";
      case FlightState::WAIT_TAKEOFF_COMPLETE:return "WAIT_TAKEOFF_COMPLETE";
      case FlightState::FLY_TO_POINT:         return "FLY_TO_POINT";
      case FlightState::RETURN_TO_BASE:       return "RETURN_TO_BASE";
      case FlightState::WAIT_RTL_RESPONSE:    return "WAIT_RTL_RESPONSE";
      case FlightState::SEND_LANDING:         return "SEND_LANDING";
      case FlightState::WAIT_LANDING_RESPONSE:return "WAIT_LANDING_RESPONSE";
      case FlightState::MISSION_COMPLETE:     return "MISSION_COMPLETE";
      case FlightState::ERROR:                return "ERROR";
    }
    return "UNKNOWN";
  }

  std::string mode_to_string(uint8_t mode) {
    switch (static_cast<int>(mode)) {
      case 0:  return "STABILIZE";
      case 1:  return "ACRO";
      case 2:  return "ALT_HOLD";
      case 3:  return "AUTO";
      case 4:  return "GUIDED";
      case 5:  return "LOITER";
      case 6:  return "RTL";
      case 7:  return "CIRCLE";
      case 9:  return "LAND";
      case 16: return "POSHOLD";
      case 21: return "SMART_RTL";
      default: return "MODE_" + std::to_string(static_cast<int>(mode));
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalController>(); // создание узла
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}