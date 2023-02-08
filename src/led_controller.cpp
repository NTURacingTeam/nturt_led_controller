#include "nturt_led_controller/led_controller.hpp"

LedController::LedController(rclcpp::NodeOptions _options)
    : Node("nturt_led_controller_node", _options),
      can_sub_(this->create_subscription<can_msgs::msg::Frame>(
          "/from_can_bus", 10,
          std::bind(&LedController::onCan, this, std::placeholders::_1))),
      rosout_sub_(this->create_subscription<rcl_interfaces::msg::Log>(
          "/rosout", 10,
          std::bind(&LedController::onRosout, this, std::placeholders::_1))),
      led_timer_(this->create_wall_timer(
          1s, std::bind(&LedController::led_callback, this))) {
  // register onShutdown to ros
  rclcpp::on_shutdown(std::bind(&LedController::onShutdown, this));

  // initiate wiringpi gpio
  wiringPiSetup();
  pinMode(LED_ROS_PIN, OUTPUT);
  pinMode(LED_CAN_PIN, OUTPUT);
  pinMode(LED_SIGNAL_PIN, OUTPUT);
  pinMode(LED_WARN_PIN, OUTPUT);
  pinMode(LED_ERROR_PIN, OUTPUT);
}

void LedController::onShutdown() {
  digitalWrite(LED_ROS_PIN, LOW);
  digitalWrite(LED_CAN_PIN, LOW);
  digitalWrite(LED_SIGNAL_PIN, LOW);
  digitalWrite(LED_WARN_PIN, LOW);
  digitalWrite(LED_ERROR_PIN, LOW);
}

void LedController::onCan(const std::shared_ptr<can_msgs::msg::Frame> _msg) {
  (void)_msg;

  digitalWrite(LED_CAN_PIN, HIGH);
  rclcpp::sleep_for(1ms);
  digitalWrite(LED_CAN_PIN, LOW);
}

void LedController::onRosout(
    const std::shared_ptr<rcl_interfaces::msg::Log> _msg) {
  // warn level = 30
  if (_msg->level == 30) {
    digitalWrite(LED_WARN_PIN, HIGH);
    rclcpp::sleep_for(100ms);
    digitalWrite(LED_WARN_PIN, LOW);
  }
  // error level = 40, fatal level = 50
  else if (_msg->level == 40 || _msg->level == 50) {
    digitalWrite(LED_ERROR_PIN, HIGH);
    rclcpp::sleep_for(100ms);
    digitalWrite(LED_ERROR_PIN, LOW);
  }
}

void LedController::led_callback() {
  if (!led_on_) {
    digitalWrite(LED_ROS_PIN, HIGH);
    led_on_ = true;
  } else {
    digitalWrite(LED_ROS_PIN, LOW);
    led_on_ = false;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LedController)
