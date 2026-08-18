#include <rclcpp/rclcpp.hpp> // База для ROS 2 узлов
#include <mavros_msgs/msg/state.hpp> // Для подписки на MAVROS state
#include <mavros_msgs/srv/set_mode.hpp> // Для переключения режима
#include <mavros_msgs/srv/command_bool.hpp> // Для армирования
#include <mavros_msgs/srv/command_tol.hpp> // Для взлёта
#include <memory> // для умных указателей
#include <future> // для асинхронных вызовов
#include <string>
#include <chrono>

using namespace std::chrono_literals; // Для записи 2s, 500ms и т.д.

class MinimalController : public rclcpp::Node
{
  public:

    enum class MissionStep {
      IDLE,
      WAIT_FOR_CONNECTION,
      SET_MODE,
      ARM,
      TAKEOFF,
      FLY_TO_POINT,
      LAND,
      COMPLETE,
      ERROR
    };

    MinimalController() : Node("minimal_controller")
    {
// 1. MAVROS state subscription
      state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
          current_state_ = *msg;   // короткий, неблокирующий колбэк
          RCLCPP_INFO(this->get_logger(), "State: connected=%d, armed=%d, mode=%s",
                  msg->connected, msg->armed, msg->mode.c_str());
      });
// 2. Mission control timer
    timer_ = this->create_wall_timer(100ms, [this]() { this->control_loop(); });
// 3.1. --- Клиент для установки режима GUIDED ---
      set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
      while (!set_mode_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;  // Выход из функции/узла
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
      }
      RCLCPP_INFO(this->get_logger(), "Service /mavros/set_mode is available.");

// 3.2. --- Клиент для ARM ---
      arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
      while (!arming_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;  // Выход из функции/узла
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/cmd/arming service...");
      }
      RCLCPP_INFO(this->get_logger(), "Service /mavros/cmd/arming is available.");

// 3.3. --- Клиент для взлёта ---
      takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
      while (!takeoff_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;  // Выход из функции/узла
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/cmd/takeoff service...");
      }
      RCLCPP_INFO(this->get_logger(), "Service /mavros/cmd/takeoff is available.");
    }

  private:

    void control_loop() {
      switch (mission_step_) {
        case MissionStep::IDLE:
          if (current_state_.connected) {
            mission_step_ = MissionStep::SET_MODE;
          }
          break;
        case MissionStep::WAIT_FOR_CONNECTION:
          if (current_state_.connected) {
            mission_step_ = MissionStep::SET_MODE;
          }
          break;
        case MissionStep::SET_MODE:
          if (!set_mode_sent_) {
            send_set_mode();
            set_mode_sent_ = true;
          }  
          break;
        case MissionStep::ARM:
          if (!arm_sent_) {
            send_arm();
            arm_sent_ = true;
          }  
          break;
        case MissionStep::TAKEOFF:
          if (!takeoff_sent_) {
            send_takeoff();
            takeoff_sent_ = true;
          }  
          break;
        case MissionStep::FLY_TO_POINT:
          break;
        case MissionStep::LAND:
          break;       
        case MissionStep::COMPLETE:
          break;       
        case MissionStep::ERROR:
          break;          
          // ... другие шаги на будущее
      }
      return;
    }

    void send_set_mode() {
      // 1. Переключение в режим GUIDED
      auto req_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req_mode->custom_mode = "GUIDED";
      set_mode_client_->async_send_request(req_mode,
        [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
          auto res_mode = future.get();
          if (res_mode->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "SetMode GUIDED success");
            mission_step_ = MissionStep::ARM;
          } else {
            RCLCPP_ERROR(this->get_logger(), "SetMode GUIDED rejected");
            mission_step_ = MissionStep::ERROR;
          }
        }
      );
    }
    void send_arm() {
      // 2. ARM
      auto req_arm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req_arm->value = true;
      arming_client_->async_send_request(req_arm, 
        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
          auto res_arm = future.get();
          if (res_arm->success) {
            RCLCPP_INFO(this->get_logger(), "Drone armed successfully");
            mission_step_ = MissionStep::TAKEOFF;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Drone arming failed");
            mission_step_ = MissionStep::ERROR;
          }
        }
      );
    }
    void send_takeoff() {
      // 3. Взлёт сразу после ARM
      auto req_takeoff = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      req_takeoff->altitude = alt;
      takeoff_client_->async_send_request(req_takeoff, 
        [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
          auto res_takeoff = future.get();
          if (res_takeoff->success) {
            RCLCPP_INFO(this->get_logger(), "Drone took off");
            mission_step_ = MissionStep::COMPLETE;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed");
            mission_step_ = MissionStep::ERROR;
          }
        }
      );
    }
    
    double alt = 5.0;
    MissionStep mission_step_ = MissionStep::IDLE;
    mavros_msgs::msg::State current_state_;
    bool mission_started_ = false;
    bool set_mode_sent_ = false;
    bool arm_sent_ = false;
    bool takeoff_sent_ = false;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalController>(); // create node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}