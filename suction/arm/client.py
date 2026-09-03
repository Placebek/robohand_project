"""Обёртка над Dobot TCP/IP SDK под задачи стенда.

Зачем поверх готового SDK ещё слой: у SDK есть несколько мест, где наивное
использование даёт молча неверные числа, и все они уже стоили времени.

1. `GetAngle()` и `GetPose()` отдают величину, привязанную к уставке. После
   возврата в ту же точку они печатают расхождение ровно 0.0000° по всем шести
   суставам — красиво и бессмысленно. Реальные показания энкодеров лежат в
   потоке состояния на порту 30004: `q_actual`, `tool_vector_actual`. Здесь
   позы читаются ТОЛЬКО оттуда.
2. Пока управление не переключено в TCP-режим, контроллер отвечает
   `Control Mode Is Not Tcp` на любой запрос, включая чтение. Похоже на битый
   парсер, а это режим. `connect()` вызывает `RequestControl()` сам.
3. Захват ES01 сидит на `ToolDO1` — выход на фланце, не на клеммнике
   контроллера. Мгновенная команда, работает при обесточенных суставах, так что
   продувку и замеры фронта можно гонять на неподвижной руке.
4. Мгновенные команды разъезжаются на 3-6 мс с хвостом до 13 (замерено). Для
   меток времени это много, см. docs/STATUS.md §2б.

Использование:

    from suction.arm.client import ArmClient

    with ArmClient() as arm:
        print(arm.joints())          # реальные углы, q_actual
        arm.gripper(True)            # без запитывания суставов
        arm.gripper(False)

        arm.enable()                 # ЗДЕСЬ РУКА ОЖИВАЕТ
        arm.speed(10)
        arm.move_joints([0, 0, 0, 0, 0, 0])
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT / "vendor" / "dobot_tcpip_v4") not in sys.path:
    sys.path.insert(0, str(ROOT / "vendor" / "dobot_tcpip_v4"))

from suction.config import (ARM_IP, ARM_DASHBOARD_PORT, ARM_FEEDBACK_PORT,
                            ES01_MASS_KG, ES01_OUTPUT_INDEX)

MODE_DISABLED = 4
MODE_ENABLED_IDLE = 5
MODE_ERROR = 9
MODE_NAMES = {
    0: "неизвестен", 1: "инициализация", 2: "тормоз отпущен", 3: "обесточен",
    4: "не запитан", 5: "запитан, простой", 6: "ручное ведение",
    7: "выполняет программу", 8: "одиночное движение", 9: "АВАРИЯ",
    10: "пауза", 11: "джог",
}


class ArmError(RuntimeError):
    """Контроллер отклонил команду или ушёл в аварию."""


def _err_id(resp):
    """Первое число ответа. 0 — команда принята."""
    try:
        return int(str(resp).split(",", 1)[0])
    except (ValueError, IndexError):
        return None


def _payload(resp):
    """Содержимое фигурных скобок ответа."""
    text = str(resp)
    if "{" not in text:
        return text
    return text[text.index("{") + 1:text.rindex("}")]


def _check(resp, what):
    if _err_id(resp) != 0:
        raise ArmError(f"{what}: {resp}")
    return resp


class ArmClient:
    """Dobot Magician E6. Позы читаются из потока состояния, не из dashboard."""

    def __init__(self, ip=ARM_IP, dashboard_port=ARM_DASHBOARD_PORT,
                 feedback_port=ARM_FEEDBACK_PORT, load_kg=ES01_MASS_KG):
        self.ip = ip
        self._ports = (dashboard_port, feedback_port)
        self.load_kg = load_kg
        self._robot = None
        self._enabled = False

    # --- соединение -----------------------------------------------------
    def connect(self, feedback_timeout=5.0):
        from dobot_sdk import DobotRobot
        dash, fb = self._ports
        self._robot = DobotRobot(self.ip, dashboard_port=dash, feedback_port=fb)
        self._robot.Connect()
        _check(self._rc.RequestControl(), "RequestControl")
        self._robot.StartFeedbackMonitor()

        deadline = time.monotonic() + feedback_timeout
        while self._robot.GetStatus() is None:
            if time.monotonic() > deadline:
                raise ArmError(f"поток состояния (порт {fb}) молчит "
                               f"{feedback_timeout:g} с")
            time.sleep(0.05)
        return self

    def close(self):
        if self._robot is None:
            return
        try:
            if self._enabled:
                self.disable()
        finally:
            try:
                self._robot.StopFeedbackMonitor()
            finally:
                self._robot.Disconnect()
                self._robot = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
        return False

    @property
    def _rc(self):
        if self._robot is None:
            raise ArmError("нет соединения — вызови connect() или используй with")
        return self._robot.robot_control

    def _status(self):
        st = self._robot.GetStatus() if self._robot else None
        if st is None:
            raise ArmError("поток состояния пуст")
        return st

    # --- чтение ---------------------------------------------------------
    def joints(self):
        """Углы суставов с энкодеров, градусы. Не GetAngle."""
        return list(self._status().joint_state.q_actual)

    def pose(self):
        """Декартова поза фланца [x, y, z, rx, ry, rz], реальная. Не GetPose."""
        return self._status().tool_vector_actual.to_list()

    def mode(self):
        return int(self._status().robot_mode)

    def mode_name(self):
        return MODE_NAMES.get(self.mode(), "?")

    def errors(self):
        """Список кодов ошибок; пустой — всё чисто."""
        raw = _payload(self._rc.GetErrorID()).strip()
        digits = [p for p in raw.replace("[", " ").replace("]", " ")
                  .replace(",", " ").split() if p not in ("", "0")]
        return digits

    def is_moving(self):
        return self._status().running_status != 0

    # --- питание и движение ---------------------------------------------
    def enable(self, load_kg=None, timeout=10.0):
        """Подать питание на суставы. ПОСЛЕ ЭТОГО РУКА МОЖЕТ ПОЕХАТЬ."""
        self._rc.ClearError()
        load = self.load_kg if load_kg is None else load_kg
        _check(self._rc.EnableRobot(load=load), "EnableRobot")
        self._enabled = True
        deadline = time.monotonic() + timeout
        while self.mode() != MODE_ENABLED_IDLE:
            if time.monotonic() > deadline:
                raise ArmError(f"не вышла в режим {MODE_ENABLED_IDLE} за {timeout:g} с "
                               f"(сейчас {self.mode()} — {self.mode_name()})")
            time.sleep(0.05)
        return self

    def disable(self):
        try:
            self._rc.DisableRobot()
        finally:
            self._enabled = False

    def speed(self, factor):
        """Глобальный масштаб скорости, 1-100 %."""
        if not 1 <= factor <= 100:
            raise ValueError("скорость вне 1-100")
        return _check(self._rc.SpeedFactor(int(factor)), "SpeedFactor")

    def wait_idle(self, timeout=60.0, settle=0.3):
        """Дождаться опустевшей очереди. Возвращает секунды ожидания."""
        start = time.monotonic()
        moved = False
        while time.monotonic() - start < timeout:
            m = self.mode()
            if m == MODE_ERROR:
                raise ArmError(f"авария во время движения, коды: {self.errors()}")
            if self.is_moving():
                moved = True
            elif moved or time.monotonic() - start > settle:
                return time.monotonic() - start
            time.sleep(0.02)
        raise ArmError(f"движение не завершилось за {timeout:g} с")

    def move_joints(self, angles, wait=True, **kw):
        """Абсолютное движение в координатах суставов, градусы."""
        from dobot_sdk import CoordinateType
        if len(angles) != 6:
            raise ValueError("нужно 6 углов")
        _check(self._robot.motion.MovJ(pose=list(angles),
                                       coord_type=CoordinateType.JOINT, **kw),
               "MovJ(joint)")
        return self.wait_idle() if wait else None

    def move_rel_joints(self, offsets, wait=True, **kw):
        """Относительное движение по суставам, градусы."""
        if len(offsets) != 6:
            raise ValueError("нужно 6 смещений")
        _check(self._robot.motion.RelJointMovJ(*offsets, **kw), "RelJointMovJ")
        return self.wait_idle() if wait else None

    def move_pose(self, pose, linear=False, wait=True, **kw):
        """Движение в декартовых координатах [x, y, z, rx, ry, rz]."""
        from dobot_sdk import CoordinateType
        if len(pose) != 6:
            raise ValueError("нужно 6 координат")
        fn = self._robot.motion.MovL if linear else self._robot.motion.MovJ
        _check(fn(pose=list(pose), coord_type=CoordinateType.CARTESIAN, **kw),
               "MovL" if linear else "MovJ")
        return self.wait_idle() if wait else None

    def stop(self):
        return self._rc.Stop()

    # --- захват ---------------------------------------------------------
    def gripper(self, on):
        """ES01 на ToolDO1. Работает и при обесточенных суставах."""
        return _check(self._robot.io.ToolDOInstant(ES01_OUTPUT_INDEX,
                                                   1 if on else 0),
                      "ToolDOInstant")

    def gripper_state(self):
        return _payload(self._robot.io.GetToolDO(ES01_OUTPUT_INDEX)).strip() == "1"
