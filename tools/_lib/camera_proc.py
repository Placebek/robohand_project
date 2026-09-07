#!/usr/bin/env python3
"""Поиск и остановка процессов драйвера камеры.

Почему не pgrep. `pgrep -f` сравнивает со ВСЕЙ командной строкой и ловит любой
процесс, где искомая строка просто упомянута, — включая нашу же оболочку,
если текст скрипта содержит это имя. На этом уже дважды обожглись: сначала на
`ascamera_node`, потом на `hp60c.launch.py`. Здесь /proc разбирается вручную и
проверяется структура argv, а не подстрока.

Почему мало погасить узел. Над `ascamera_node` стоит `ros2 launch`, который
поднимает его заново с новым pid. Гасить надо сначала запускающий процесс —
ровно то, что делает Ctrl+C в его окне.

Сигнал шлётся ГРУППЕ процессов, а не одному pid. Ctrl+C в терминале делает
именно это, и только так `ros2 launch` сворачивается вместе со своими узлами:
если сигнал получил один launch, узел продолжает жить и поднимается заново с
новым pid — на этом обожглись.

В launch-файле HP60C стоит `respawn=True`: launch намеренно поднимает узел
заново каждый раз, когда тот умирает. Поэтому гасить узел бесполезно в принципе
— надо остановить сам launch.

Лестница эскалации, снизу вверх:
  1. SIGINT группе — как Ctrl+C. Узел умирает, но launch у нас так и не
     сворачивался: он реагирует на этот сигнал, только когда владеет терминалом.
  2. SIGTERM самому launch. Он принимает его как просьбу завершиться и сам
     аккуратно гасит свои узлы, давая им отработать обработчики.
  3. SIGINT осиротевшим узлам. Когда launch мёртв, respawn над ними уже нет,
     и узел спокойно доживает до конца обработчиков.
  4. сдаться и попросить человека закрыть окно.

SIGKILL не шлём НИКОГДА: SDK работает через libusb напрямую и должен освободить
устройство сам, иначе следующий запуск поднимется с мёртвой областью.

Запуск:
  python3 tools/_lib/camera_proc.py list
  python3 tools/_lib/camera_proc.py stop [таймаут]
"""
import os
import signal
import sys
import time
from pathlib import Path


def _argv(pid):
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (OSError, ValueError):
        return None
    return [a for a in raw.decode("utf-8", "replace").split("\0") if a]


def _is_node(argv):
    return bool(argv) and Path(argv[0]).name == "ascamera_node"


def _is_launcher(argv):
    # /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch ascamera hp60c.launch.py
    # Проверяем ЭЛЕМЕНТЫ argv, а не подстроку: у `bash -c ...` вся команда
    # лежит одним куском в argv[2] и такой проверки не проходит.
    if len(argv) < 4:
        return False
    if not any(Path(a).name == "ros2" for a in argv):
        return False
    return "launch" in argv and "ascamera" in argv


def driver_pids():
    """(запускающие процессы, узлы). Свой процесс и предков исключаем."""
    mine = {os.getpid(), os.getppid()}
    launchers, nodes = [], []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in mine:
            continue
        argv = _argv(pid)
        if argv is None:
            continue
        if _is_launcher(argv):
            launchers.append(pid)
        elif _is_node(argv):
            nodes.append(pid)
    return sorted(launchers), sorted(nodes)


def _signal(pid, sig=signal.SIGINT, group=True):
    """Послать сигнал процессу или его группе.

    Свою собственную группу не трогаем никогда — иначе прибьём сами себя
    вместе с вызвавшей оболочкой.
    """
    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        return
    try:
        if group and pgid != os.getpgid(0):
            os.killpg(pgid, sig)
        else:
            os.kill(pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def _wait_gone(timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if driver_pids() == ([], []):
            return True
        time.sleep(0.4)
    return driver_pids() == ([], [])


def stop(timeout=20.0):
    """Погасить драйвер по-хорошему. True — всё закрылось."""
    launchers, nodes = driver_pids()
    if not launchers and not nodes:
        return True

    # 1. SIGINT группе — то же, что Ctrl+C в окне драйвера.
    for pid in launchers + nodes:
        _signal(pid, signal.SIGINT)
    if _wait_gone(timeout * 0.4):
        return True

    # 2. SIGTERM самому launch. Узлам его не шлём: пусть launch свернёт их сам,
    # тогда SDK успеет освободить устройство.
    launchers, _ = driver_pids()
    if launchers:
        print("  launch не отозвался на SIGINT, шлю SIGTERM ему самому")
        for pid in launchers:
            _signal(pid, signal.SIGTERM, group=False)
        if _wait_gone(timeout * 0.3):
            return True

    # 3. Узел, воскрешённый за миг до смерти launch, остаётся сиротой. Теперь
    # над ним нет respawn, и SIGINT ему уже не бесполезен.
    _, orphans = driver_pids()
    if orphans:
        print(f"  добиваю осиротевший узел: {orphans}")
        for pid in orphans:
            _signal(pid, signal.SIGINT, group=False)
        if _wait_gone(timeout * 0.3):
            return True
    return driver_pids() == ([], [])


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "list"
    launchers, nodes = driver_pids()
    if what == "list":
        if not launchers and not nodes:
            print("драйвер не запущен")
            return 1
        if launchers:
            print(f"ros2 launch: {', '.join(map(str, launchers))}")
        if nodes:
            print(f"ascamera_node: {', '.join(map(str, nodes))}")
        return 0
    if what == "stop":
        if not launchers and not nodes:
            print("  драйвер не запущен")
            return 0
        print(f"  останавливаю драйвер (SIGINT): "
              f"launch {launchers or '-'}, node {nodes or '-'}")
        t = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
        if stop(t):
            print("  остановлен")
            return 0
        left = driver_pids()
        print(f"  НЕ ЗАКРЫЛСЯ за {t:g} с, осталось: {left}", file=sys.stderr)
        print("  Убивать через kill -9 нельзя: устройство останется занятым.",
              file=sys.stderr)
        print("  Закрой его вручную Ctrl+C в его окне.", file=sys.stderr)
        return 1
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
