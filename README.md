# learn-ros2

A ROS 2 learning project exploring core concepts and development practices.

## Prerequisites

- ROS 2 Jazzy

## Installation

### Install ROS 2

Follow the official [ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation.html) for your platform.

## Building the Project

From the workspace root, build and run the sample `talker` node with these steps:

```bash
# build (with symlink install and export compile commands)
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# source the workspace overlay
source install/setup.bash

# run the node
ros2 run my_test_pkg talker
```