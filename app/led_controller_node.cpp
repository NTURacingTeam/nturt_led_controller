// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_led_controller/led_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  rclcpp::Node::SharedPtr led_controller_node =
      std::make_shared<LedController>(options);

  executor.add_node(led_controller_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
