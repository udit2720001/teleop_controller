
# Teleop Controller (ESP-IDF)

This project demonstrates how to set up and run a teleoperation controller using ESP-IDF and micro-ROS.

## Prerequisites

- ESP-IDF installed and configured ([ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
- Docker installed (for running micro-ROS agent)
- Git

## Setup Instructions

```bash
# Clone ESP-IDF and enter your workspace
cd ~/esp

# Create a new ESP-IDF project
idf.py create-project teleop_controller
cd teleop_controller

# Add micro-ROS component
mkdir -p components
cd components
git clone https://github.com/micro-ROS/micro_ros_espidf_component
cd ..

# Set target to ESP32
idf.py set-target esp32

# Configure project options
idf.py menuconfig
```

## Running micro-ROS Agent

Start the micro-ROS agent using Docker:

```bash
sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v4
```

## Notes

- Adjust the UDP port as needed (default: 8888).
- Refer to the micro-ROS documentation for further configuration.
