#include <rclcpp/rclcpp.hpp> // База для ROS 2 узлов
#include <cmath> // Математика
#include <vector> // Вектора
#include <memory> // Для умных указателей
// #include <future> // На будущее
#include <string> // Строковые переменные
#include <chrono> // Временные библиотеки

// Интерфейсы ArduPilot
#include <ardupilot_msgs/srv/arm_motors.hpp>
#include <ardupilot_msgs/srv/takeoff.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>
#include <ardupilot_msgs/msg/status.hpp>
#include <ardupilot_msgs/msg/global_position.hpp>

// Интерфейсы ROS 2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

using namespace std::chrono_literals; // Для записи 2s, 500ms и т.д.

class MinimalController : public rclcpp::Node {
public:

  enum class FlightState {
    INIT,
    SET_GUIDED_MODE,
    WAIT_GUIDED_RESPONSE,
    SEND_ARMING,
    WAIT_ARMING_RESPONSE,
    SEND_TAKEOFF,
    WAIT_TAKEOFF_COMPLETE,
    FLY_TO_POINT,
    SEND_LANDING,
    WAIT_LANDING_RESPONSE,
    MISSION_COMPLETE,
    ERROR
  };

  struct Waypoint {
    double lat;
    double lon;
    float alt;
  };

  MinimalController() : Node("minimal_controller"), current_state_(FlightState::INIT),
                        target_waypoint_idx_(0) 
  {
// 1. Клиенты сервисов ArduPilot
  mode_client_ = this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");
  arming_client_ = this->create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");
  takeoff_client_ = this->create_client<ardupilot_msgs::srv::Takeoff>("/ap/experimental/takeoff");

// 2. Издание и подписка
  setpoint_pub_ = this->create_publisher<ardupilot_msgs::msg::GlobalPosition>("/ap/cmd_gps_pose", 10);

  auto telemetry_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  pose_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
      "/ap/geopose/filtered", telemetry_qos,
      [this](const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
        this->pose_telemetry_callback(msg);
      }
  );

// 3. Точки облета

  waypoints_ = {
    {home_lat, home_lon, alt},                      // Взлёт в эту точку
    {home_lat + 0.0009, home_lon, alt},             // 100 м на север
    {home_lat + 0.0009, home_lon + 0.0009, alt},    // 100 м на восток
    {home_lat, home_lon + 0.0009, alt},             // 100 м на юг
    {home_lat, home_lon, alt}                       // Возврат
  };

// 4. Таймер автомата (2 Гц)
  timer_ = this->create_wall_timer(500ms, [this]() { this->control_loop(); });

  RCLCPP_INFO(this->get_logger(), "Контроллер запущен в один поток.");
  }

private:

  void pose_telemetry_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
    current_lat_ = msg->pose.position.latitude;
    current_lon_ = msg->pose.position.longitude;
    current_alt_ = msg->pose.position.altitude;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "[ТЕЛЕМЕТРИЯ] XYZ: [%.7f, %.7f, %.7f]", current_lat_, current_lon_, current_alt_);

    if (current_state_ == FlightState::FLY_TO_POINT) {
        check_waypoint_arrival();
    }
  }

  void control_loop() {
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
        RCLCPP_INFO(this->get_logger(), "Запрос взлета на 5 метров...");
        send_takeoff_request(5.0f);
        current_state_ = FlightState::WAIT_TAKEOFF_COMPLETE;
        break;
      case FlightState::WAIT_TAKEOFF_COMPLETE:
        check_waypoint_arrival();
        break;
      case FlightState::FLY_TO_POINT:
        publish_target_waypoint();
        break;
      case FlightState::SEND_LANDING:
        RCLCPP_INFO(this->get_logger(), "Запрос режима LAND (9)...");
        send_mode_request(9);
        current_state_ = FlightState::WAIT_LANDING_RESPONSE;
        break;
      case FlightState::WAIT_LANDING_RESPONSE:
        break;
      case FlightState::MISSION_COMPLETE:
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Миссия завершена успешно!");
        break;
      case FlightState::ERROR:
        break;          
// ... другие шаги на будущее
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
        else if (current_state_ == FlightState::WAIT_LANDING_RESPONSE) current_state_ = FlightState::MISSION_COMPLETE;
    } else {
        if (current_state_ == FlightState::WAIT_GUIDED_RESPONSE) current_state_ = FlightState::SET_GUIDED_MODE;
        else if (current_state_ == FlightState::WAIT_LANDING_RESPONSE) current_state_ = FlightState::SEND_LANDING;
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

  double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;                      // радиус Земли в метрах
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180.0) * 
               cos(lat2 * M_PI / 180.0) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
  }

  void check_waypoint_arrival() {
    if (target_waypoint_idx_ >= waypoints_.size()) return;
    auto target = waypoints_[target_waypoint_idx_];
    double horizontal_dist = haversine(current_lat_, current_lon_, target.lat, target.lon);
    double vertical_dist = std::abs((current_alt_ - home_alt) - target.alt);
    if (current_state_ == FlightState::WAIT_TAKEOFF_COMPLETE && vertical_dist < 0.5) current_state_ = FlightState::FLY_TO_POINT;
    if (current_state_ == FlightState::FLY_TO_POINT && horizontal_dist < 0.6) {
      target_waypoint_idx_++;
      RCLCPP_INFO(this->get_logger(), "Запрос полёта в следующую точку...");
      if (target_waypoint_idx_ >= waypoints_.size()) current_state_ = FlightState::SEND_LANDING;
    }
  }

// Состояние автомата и миссии
  FlightState current_state_;
  std::vector<Waypoint> waypoints_;
  size_t target_waypoint_idx_;

// Координаты дрона
  double current_lat_{0.0};
  double current_lon_{0.0};
  double current_alt_{0.0};

// Пример для home с координатами аэропорта Канберры
  double home_lat = -35.36326217651367;
  double home_lon = 149.1652374267578;
  double home_alt = 584.09;

  float alt = 5.0f; // Начальная высота от точки старта

// ROS 2 интерфейсы
  rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr mode_client_;
  rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr arming_client_;
  rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedPtr takeoff_client_;
    
  rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr pose_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalController>(); // create node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}