# Предполётная подготовка: камера отдаёт кадры, рука без аварии.
# Подключается через source после tools/_lib/env.sh.
#
# Оба отказа, от которых это защищает, МОЛЧАЛИВЫЕ. HP60C после перезапуска
# драйвера поверх не отпущенного устройства поднимается с мёртвой областью — до
# 17 % рабочего круга приходит нулями без ошибок в лог. Рука после срабатывания
# защиты не принимает ни движений, ни ручного ведения, пока аварию не снять.
# Наткнуться на любое посреди калибровки — потерять всю серию поз.
#
# Здоровый работающий драйвер НЕ трогаем: перезапуск стоит полминуты и сам по
# себе способен испортить состояние устройства.

CAMERA_LOG="/tmp/robohand-camera.log"
CAMERA_VID_PID="3482:6723"

_py() { env "${ROBOHAND_ENV[@]}" python3 "$@"; }

_camera_on_bus() { lsusb -d "$CAMERA_VID_PID" >/dev/null 2>&1; }

# 0 здорова, 1 нулей больше порога, 2 кадров нет.
# Долю нулей кладёт в CAMERA_ZERO_PCT — по ней сравниваем до и после перезапуска.
_camera_health() {
    local out rc=0
    out="$(_py "$ROBOHAND_ROOT/tools/_lib/camera_health.py" 2>&1)" || rc=$?
    CAMERA_ZERO_PCT="$(printf '%s\n' "$out" | sed -n 's/^PCT=//p')"
    printf '%s\n' "$out" | grep -v '^PCT='
    return $rc
}

_start_camera() {
    echo "  запускаю драйвер, лог: $CAMERA_LOG"
    # setsid — своя группа процессов. Тогда его можно погасить целиком, как
    # Ctrl+C, не задев собственную оболочку.
    setsid nohup "$ROBOHAND_ROOT/tools/camera/start.sh" >"$CAMERA_LOG" 2>&1 &
    sleep 2
    if ! _py "$ROBOHAND_ROOT/tools/_lib/wait_camera.py" 40; then
        echo "  камера не отдаёт кадры. Смотри $CAMERA_LOG" >&2
        return 1
    fi
}

_recycle_camera() {   # погасить -> сбросить USB -> поднять
    _py "$ROBOHAND_ROOT/tools/_lib/camera_proc.py" stop 20 || return 1
    _py "$ROBOHAND_ROOT/tools/camera/reset.py" --quiet || return 1
    _start_camera || return 1
}

# robohand_preflight [камера|рука|всё] [--depth-critical]
# --depth-critical: мёртвая область — повод остановиться. Без флага только
# предупреждение: для hand-eye глубина не нужна, съёмка поз идёт по RGB.
robohand_preflight() {
    local what="${1:-всё}" strict=0
    [ "${2:-}" = "--depth-critical" ] && strict=1

    if [ "$what" = "всё" ] || [ "$what" = "камера" ]; then
        echo "== камера =="
        if ! _camera_on_bus; then
            echo "  камеры $CAMERA_VID_PID нет на шине USB." >&2
            echo "  Проверь кабель. Если только что дёргали — подожди пару секунд." >&2
            return 1
        fi

        if _py "$ROBOHAND_ROOT/tools/_lib/camera_proc.py" list >/dev/null 2>&1; then
            echo "  драйвер уже работает, проверяю"
            local rc=0 before=""
            _camera_health || rc=$?
            before="$CAMERA_ZERO_PCT"
            if [ $rc -ne 0 ]; then
                echo "  поднимаю заново"
                _recycle_camera || return 1
                local after=""
                if ! _camera_health; then
                    after="$CAMERA_ZERO_PCT"
                    # Мёртвая область после перезапуска исчезает, тёмный предмет
                    # остаётся. Если доля почти та же — дело в сцене.
                    local same
                    same=$(awk -v a="$before" -v b="$after" \
                        'BEGIN{d=a-b; if(d<0)d=-d; print (a!="" && b!="" && d<3)?1:0}')
                    if [ "$same" = "1" ]; then
                        echo "  доля нулей не изменилась ($before% -> $after%):"
                        echo "  это ТЁМНЫЕ ПРЕДМЕТЫ в кадре, а не мёртвая область."
                        echo "  Чёрное не отражает ИК. Камера в порядке."
                    elif [ $strict -eq 1 ]; then
                        echo "  мёртвая область осталась и после сброса." >&2
                        return 1
                    else
                        echo "  нули остались. Для hand-eye не критично:"
                        echo "  съёмка поз идёт по RGB, глубина в ней не участвует."
                    fi
                fi
            fi
        else
            echo "  драйвер не запущен"
            _py "$ROBOHAND_ROOT/tools/camera/reset.py" --quiet || return 1
            _start_camera || return 1
            _camera_health || true
        fi
    fi

    if [ "$what" = "всё" ] || [ "$what" = "рука" ]; then
        echo
        echo "== рука =="
        env -u LD_LIBRARY_PATH -u PYTHONPATH "${_SNAP_VARS[@]}" \
            python3 "$ROBOHAND_ROOT/tools/arm/clear_fault.py" || return 1
    fi
    echo
}

# robohand_camera_ready [--verify-zone]
#
# Приводит камеру в рабочее состояние перед любым инструментом, который её
# использует: драйвер поднят, мёртвой области нет.
#
# --verify-zone дополнительно сверяет положение камеры с эталоном
# data/calib/zone_baseline.json и ОСТАНАВЛИВАЕТ работу, если камеру сдвинули.
# Нужен там, где используется калибровка: сдвиг камеры не проявляется ничем —
# матрица на месте, точки переводятся, рука едет мимо.
robohand_camera_ready() {
    robohand_preflight камера || return 1
    if [ "${1:-}" = "--verify-zone" ]; then
        echo "== положение камеры =="
        if ! _py "$ROBOHAND_ROOT/tools/camera/zone_check.py" --quiet; then
            echo "  Продолжать нельзя: результат будет молча неверным." >&2
            echo "  Либо верни камеру на место, либо пройди калибровку заново" >&2
            echo "  (docs/calibration.md). Обойти проверку: --skip-preflight" >&2
            return 1
        fi
        echo
    fi
}
