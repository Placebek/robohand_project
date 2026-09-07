#!/bin/bash
# Драйвер камеры Nuwa/Angstrong HP60C (ROS 2 Jazzy).
#
# Оставь это окно открытым. Останавливать ТОЛЬКО Ctrl+C, не kill -9: SDK
# работает через libusb напрямую и должен освободить устройство сам. Иначе
# следующий запуск поднимется с мёртвой областью — часть кадра будет приходить
# нулями всегда, без ошибок в лог.
set -e
WS="/home/jake/Downloads/4.ROS2_Code-20260115T112805Z-3-001/4.ROS2_Code/ascam_ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
cd "$WS"
exec ros2 launch ascamera hp60c.launch.py
