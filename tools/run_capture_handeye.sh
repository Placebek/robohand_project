#!/bin/bash
# Съёмка поз для hand-eye. РУКА ЗАПИТЫВАЕТСЯ И СТАНОВИТСЯ ПОДАТЛИВОЙ.
# Открывает окно превью, поэтому нужен тот же набор Qt-переменных, что и у
# просмотрщика. LD_LIBRARY_PATH не трогать — на нём висит rclpy.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."
source /opt/ros/jazzy/setup.bash

exec env -u SNAP -u SNAP_LIBRARY_PATH -u SNAP_NAME -u SNAP_REVISION -u SNAP_DATA \
    -u SNAP_USER_DATA -u SNAP_USER_COMMON -u SNAP_REAL_HOME -u GTK_PATH \
    -u GTK_EXE_PREFIX -u LOCPATH -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE \
    -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR -u GTK_IM_MODULE_FILE -u XDG_DATA_DIRS \
    -u XDG_DATA_HOME -u WAYLAND_DISPLAY \
    QT_QPA_PLATFORM=xcb \
    python3 tools/capture_handeye.py "$@"
