// stl include
#include <array>
#include <iostream>
#include <utility>

// gpio include
#include <wiringPi.h>

// nturt include
#include "nturt_led_controller/project_def.hpp"

#define LED_TUPLE(LED) std::make_pair<int, const char*>((LED), (#LED))

std::array<std::pair<int, const char*>, 5> led_pins = {
    LED_TUPLE(LED_ROS_PIN), LED_TUPLE(LED_CAN_PIN), LED_TUPLE(LED_WARN_PIN),
    LED_TUPLE(LED_ERROR_PIN), LED_TUPLE(LED_RESERVED_PIN)};

int main(int /*argc*/, char** /*argc*/) {
  wiringPiSetup();
  for (auto& led_pin : led_pins) {
    pinMode(led_pin.first, OUTPUT);
  }

  while (true) {
    for (auto& led_pin : led_pins) {
      std::cout << "blinking: " << led_pin.second << std::endl;
      digitalWrite(led_pin.first, HIGH);
      delay(2000);
      digitalWrite(led_pin.first, LOW);
    }
  }
  return 0;
}
