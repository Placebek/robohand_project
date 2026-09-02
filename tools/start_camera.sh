#!/bin/bash
# Запускает ROS2-драйвер камеры Nuwa/YDLIDAR HP60C.
# Оставь это окно открытым, останавливай по Ctrl+C (не kill -9!) —
# SDK камеры работает через libusb напрямую и должен корректно освободить устройство.
set -e
WS="/home/jake/Downloads/4.ROS2_Code-20260115T112805Z-3-001/4.ROS2_Code/ascam_ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
cd "$WS"
exec ros2 launch ascamera hp60c.launch.py
