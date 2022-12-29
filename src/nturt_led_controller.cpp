#include "nturt_led_controller/nturt_led_controller.hpp"

LedController::LedController() : Node("nturt_lled_controller_node"),
    can_sub_(this->create_subscription<can_msgs::msg::Frame>("/received_messages", 10, std::bind(&LedController::onCan, this, std::placeholders::_1))),
    rosout_sub_(this->create_subscription<rcl_interfaces::msg::Log>("/rosout", 10, std::bind(&LedController::onRosout, this, std::placeholders::_1))),
    led_timer_(this->create_wall_timer(1s, std::bind(&LedController::led_callback, this))) {
    
    // initiate wiringpi gpio
    wiringPiSetup();
    pinMode(LED_ROS_PIN, OUTPUT);
    pinMode(LED_CAN_PIN, OUTPUT);
    pinMode(LED_SIGNAL_PIN, OUTPUT);
    pinMode(LED_WARN_PIN, OUTPUT);
    pinMode(LED_ERROR_PIN, OUTPUT);
}

void LedController::cleanup() {
    digitalWrite(LED_ROS_PIN, LOW);
    digitalWrite(LED_CAN_PIN, LOW);
    digitalWrite(LED_SIGNAL_PIN, LOW);
    digitalWrite(LED_WARN_PIN, LOW);
    digitalWrite(LED_ERROR_PIN, LOW);
}

void LedController::onCan(const std::shared_ptr<can_msgs::msg::Frame> /*_msg*/) {
    digitalWrite(LED_CAN_PIN, HIGH);
    rclcpp::sleep_for(1ms);
    digitalWrite(LED_CAN_PIN, LOW);
}

void LedController::onRosout(const std::shared_ptr<rcl_interfaces::msg::Log> _msg) {
    if(_msg->level == 4) {
        digitalWrite(LED_WARN_PIN, HIGH);
        rclcpp::sleep_for(100ms);
        digitalWrite(LED_WARN_PIN, LOW);
    }
    else if(_msg->level == 8 || _msg->level == 16) {
        digitalWrite(LED_ERROR_PIN, HIGH);
        rclcpp::sleep_for(100ms);
        digitalWrite(LED_ERROR_PIN, LOW);
    }
}

void LedController::led_callback() {
    if(!led_on_) {
        digitalWrite(LED_ROS_PIN, HIGH);
        led_on_ = true;
    }
    else{
        digitalWrite(LED_ROS_PIN, LOW);
        led_on_ = false;
    }
}
