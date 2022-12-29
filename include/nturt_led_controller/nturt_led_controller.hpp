/**
 * @file nturt_led_controller.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 package for controlling leds connected to rpi that indicate the state of ros2.
 */

#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

// led pin out in wiringpi fasion
#define LED_ROS_PIN 25
#define LED_CAN_PIN 24
#define LED_SIGNAL_PIN 23
#define LED_WARN_PIN 28
#define LED_ERROR_PIN 27

// std include
#include <chrono>
#include <functional>
#include <memory>

// gpio include
#include <wiringPi.h>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// ros2 message include
#include "can_msgs/msg/frame.hpp"
#include "rcl_interfaces/msg/log.hpp"

using namespace std::chrono_literals;

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for controlling leds connected to rpi that indicate the state of ros2.
 */
class LedController : public rclcpp::Node {
    public:
        /// @brief Constructor of led controller.
        /// @param _nh Shared pointer to can handle.
        LedController();

        /// @brief Function that should be called before the function terminates.
        void cleanup();
    private:
        /// @brief ROS2 sbscriber to "/received_messages", for receiving can signal.
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

        /// @brief ROS2 subscriber to "/rosout". for receiving error messages from other nodes.
        rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

        /// @brief ROS2 timer for determine if ros is alive.
        rclcpp::TimerBase::SharedPtr led_timer_;

        // internal state
        /// @brief If ros led is on.
        bool led_on_ = false;

        /// @brief Callback function when receiving message from "/received_messages".
        void onCan(const std::shared_ptr<can_msgs::msg::Frame> _msg);

        /// @brief Callback function when receiving message from "/rosout".
        void onRosout(const std::shared_ptr<rcl_interfaces::msg::Log> _msg);

        /// @brief Timed callback function called for blinking ros led on and off.
        void led_callback();
};

#endif