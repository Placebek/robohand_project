# Предполётная подготовка: камера с чистого листа, рука без аварии.
# Подключается через source после tools/_lib/env.sh.
#
# Зачем: оба отказа МОЛЧАЛИВЫЕ. HP60C после перезапуска драйвера поверх не
# отпущенного устройства поднимается с мёртвой областью — до 17 % рабочего
# круга приходит нулями без единой ошибки в лог. Рука после срабатывания
# защиты не принимает ни движений, ни ручного ведения, пока аварию не снять.
# Наткнуться на любое из этого посреди калибровки — потерять всю серию поз.

CAMERA_LOG="/tmp/robohand-camera.log"

# Остановить драйвер камеры по-хорошему. НЕ kill -9: SDK работает через libusb
# напрямую и должен освободить устройство сам, иначе следующий запуск поднимется
# с мёртвой областью.
_stop_camera() {
    local pids
    pids=$(pgrep -x ascamera_node || true)
    [ -z "$pids" ] && return 0

    echo "  останавливаю драйвер камеры (SIGINT, pid $pids)"
    kill -INT $pids 2>/dev/null || true
    for _ in $(seq 1 30); do
        sleep 0.5
        pgrep -x ascamera_node >/dev/null || { echo "  остановлен"; return 0; }
    done
    echo "  ДРАЙВЕР НЕ ЗАКРЫЛСЯ за 15 с." >&2
    echo "  Убивать его через kill -9 нельзя: устройство останется занятым," >&2
    echo "  и следующий запуск поднимется с мёртвой областью." >&2
    echo "  Закрой его вручную Ctrl+C в его окне и запусти снова." >&2
    return 1
}

_start_camera() {
    echo "  запускаю драйвер, лог: $CAMERA_LOG"
    nohup "$ROBOHAND_ROOT/tools/camera/start.sh" >"$CAMERA_LOG" 2>&1 &
    sleep 2
    if ! env "${ROBOHAND_ENV[@]}" python3 "$ROBOHAND_ROOT/tools/_lib/wait_camera.py" 40; then
        echo "  Камера не отдаёт кадры. Смотри $CAMERA_LOG" >&2
        return 1
    fi
}

# robohand_preflight [камера|рука|всё]
robohand_preflight() {
    local what="${1:-всё}"

    if [ "$what" = "всё" ] || [ "$what" = "камера" ]; then
        echo "== камера с чистого листа =="
        _stop_camera || return 1
        env "${ROBOHAND_ENV[@]}" python3 "$ROBOHAND_ROOT/tools/camera/reset.py" || return 1
        _start_camera || return 1
    fi

    if [ "$what" = "всё" ] || [ "$what" = "рука" ]; then
        echo
        echo "== рука без аварии =="
        env -u LD_LIBRARY_PATH -u PYTHONPATH "${_SNAP_VARS[@]}" \
            python3 "$ROBOHAND_ROOT/tools/arm/clear_fault.py" || return 1
    fi
    echo
}
