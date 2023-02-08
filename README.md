# NTURT Led Controller

## Introduction

To better determine what's going on in ros2, leds are used to indicated the state of the ros2 system, there are currently four leds in use:

1. ros: This led will flash on and off with a frequency of $2s$ to indicate the ros2 system is running.
2. can: This led will flash once($1ms$) when receiving can signals.
3. warn: This led will flash once($100ms$) when a warning message is sent from a ros2 node.
4. error:  This led will flash once($100ms$) when an error message is sent from a ros2 node.

## Usage

This node can be run by:

```shell=
ros2 run nturt_led_controller nturt_led_controller_node
```
