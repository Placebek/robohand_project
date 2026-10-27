# Общая подготовка окружения. Подключается через source из обёрток.
#
# VS Code (snap) подмешивает в терминал переменные, которые ломают Qt-бэкенд
# OpenCV: symbol lookup error, QSocketNotifier, окно не открывается. Здесь они
# снимаются в одном месте, чтобы не расползались копиями по обёрткам.
#
# LD_LIBRARY_PATH при работе с ROS НЕ ТРОГАТЬ — на нём висит rclpy, и его
# снятие даёт "librcl_action.so: cannot open shared object file".
#
# Профили:
#   plain — рука, чистый python без ROS и без окон
#   ros   — нужен rclpy, окон нет
#   qt    — нужен rclpy и окно OpenCV

ROBOHAND_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Интерпретатор стенда берётся АБСОЛЮТНЫМ путём, а не из PATH. Иначе
# активированный в шелле .venv-lerobot (numpy 2, он нужен только для
# tools/vla/pack.sh) подменяет собой окружение стенда: cv_bridge собран под
# numpy 1.x и падает с ImportError "compiled using NumPy 1.x", а следом rclpy
# роняет процесс в segfault уже посреди работы. Поймано на первом же прогоне
# записи демонстраций.
if [ -x "$ROBOHAND_ROOT/.venv/bin/python3" ]; then
    ROBOHAND_PY="$ROBOHAND_ROOT/.venv/bin/python3"
else
    ROBOHAND_PY="python3"
    echo "внимание: .venv стенда не найден, беру python3 из PATH" >&2
fi

_SNAP_VARS=(-u SNAP -u SNAP_LIBRARY_PATH -u SNAP_NAME -u SNAP_REVISION
            -u SNAP_DATA -u SNAP_USER_DATA -u SNAP_USER_COMMON -u SNAP_REAL_HOME)

_GUI_VARS=(-u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH -u GDK_PIXBUF_MODULEDIR
           -u GDK_PIXBUF_MODULE_FILE -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR
           -u GTK_IM_MODULE_FILE -u XDG_DATA_DIRS -u XDG_DATA_HOME
           -u WAYLAND_DISPLAY)

# robohand_env <профиль> — заполняет массив ROBOHAND_ENV
robohand_env() {
    ROBOHAND_ENV=("${_SNAP_VARS[@]}")
    # Следы чужого активированного venv снимаем и в дочернем окружении: сам
    # скрипт зовётся по абсолютному пути, но всё, что он запустит, не должно
    # видеть постороннее окружение как своё. Все ключи -u идут ДО первого
    # присваивания NAME=VALUE — иначе env перестаёт разбирать их как опции.
    [ -x "$ROBOHAND_ROOT/.venv/bin/python3" ] &&
        ROBOHAND_ENV+=(-u VIRTUAL_ENV -u VIRTUAL_ENV_PROMPT)
    case "$1" in
        plain) ROBOHAND_ENV+=(-u LD_LIBRARY_PATH -u PYTHONPATH) ;;
        ros)   source /opt/ros/jazzy/setup.bash ;;
        qt)    source /opt/ros/jazzy/setup.bash
               ROBOHAND_ENV+=("${_GUI_VARS[@]}" QT_QPA_PLATFORM=xcb) ;;
        *)     echo "robohand_env: неизвестный профиль '$1'" >&2; return 2 ;;
    esac
    # PATH — уже ПОСЛЕ case: иначе явное присваивание затрёт то, что добавил
    # в него setup.bash ROS.
    [ -x "$ROBOHAND_ROOT/.venv/bin/python3" ] &&
        ROBOHAND_ENV+=("PATH=$ROBOHAND_ROOT/.venv/bin:$PATH")
    return 0
}

# robohand_run <профиль> <скрипт от корня> [аргументы...]
robohand_run() {
    local profile="$1" script="$2"; shift 2
    robohand_env "$profile" || return $?
    cd "$ROBOHAND_ROOT"
    exec env "${ROBOHAND_ENV[@]}" "$ROBOHAND_PY" "$script" "$@"
}
