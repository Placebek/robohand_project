#!/bin/bash
# Проверка калибровки от пикселя до координат руки. Ничего не двигает.
# камеры (tools/start_camera.sh). LD_LIBRARY_PATH не трогать — на нём висит rclpy.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."
source /opt/ros/jazzy/setup.bash

exec env -u SNAP -u SNAP_LIBRARY_PATH -u SNAP_NAME -u SNAP_REVISION -u SNAP_DATA \
    -u SNAP_USER_DATA -u SNAP_USER_COMMON -u SNAP_REAL_HOME -u GTK_PATH \
    -u GTK_EXE_PREFIX -u LOCPATH -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR \
    python3 tools/check_handeye.py "$@"
