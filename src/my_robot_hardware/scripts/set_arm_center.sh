#!/bin/bash

# Script to set all robot arm joints to center position (2048)
# Usage: ./set_arm_center.sh [serial_port]

SERIAL_PORT=${1:-/dev/ttyUSB0}

echo "Setting robot arm joints to center position (2048)..."
echo "Serial port: $SERIAL_PORT"

ros2 run my_robot_hardware set_arm_center $SERIAL_PORT