# NTURT Led Controller

## Introduction

To better determine what's going on in ros2, leds are used to indicated the state of the ros2 system, there are currently four leds in use:

1. ros: This LED will flash on and off with a frequency of $2s$ to indicate the ros2 system is running.
2. can: This LED will flash once (1ms) when receiving can signals.
3. warn: This LED will flash once (100ms) when a warning message is sent from a ros2 node.
4. error: This LED will flash once (100ms) when an error/fatal message is sent from a ros2 node.

## Usage

This node can be run by:

```shell=
ros2 run nturt_led_controller nturt_led_controller_node
```

### led_test

This package also provides test to check if all LEDs are correctly connected/configured. It will first print the name of a LED and turn it on for 5s and proceed to the next one in the following order:

1. ros
2. can
3. warn
4. error
5. reserved

Usage:

```
ros2 run nturt_led_controller led_test
```
