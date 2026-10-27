#!/usr/bin/env python3
"""Windows GUI for teaching/replaying Dobot Magician E6 joint poses."""
import json
import queue
import sys
import threading
import time
import traceback
import tkinter as tk
from datetime import datetime, timezone
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from suction.arm.client import ArmClient, ArmError
    from suction.config import ES01_MASS_KG
except Exception as exc:
    ArmClient = None
    ArmError = RuntimeError
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

SCHEMA_VERSION = 1
DEFAULT_PROGRAM = ROOT / "programs" / "pick_and_place.json"


def validate_points(points):
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("В программе должно быть минимум две точки.")
    result = []
    for i, p in enumerate(points, 1):
        if not isinstance(p, dict) or not str(p.get("name", "")).strip():
            raise ValueError(f"Точка {i}: отсутствует имя.")
        for key in ("joints", "pose"):
            v = p.get(key)
            if not isinstance(v, list) or len(v) != 6:
                raise ValueError(f"Точка {i}: {key} должен содержать 6 чисел.")
            p[key] = [float(x) for x in v]
        if not isinstance(p.get("vacuum"), bool):
            raise ValueError(f"Точка {i}: vacuum должен быть true/false.")
        result.append({"name": str(p["name"]), "joints": p["joints"], "pose": p["pose"], "vacuum": p["vacuum"]})
    return result


class RoboHandApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RoboHand — Dobot E6 Teach & Repeat")
        self.geometry("1050x720")
        self.minsize(900, 620)
        self.arm = None
        self.busy = False
        self.teaching = False
        self.vacuum = False
        self.points = []
        self.program_path = DEFAULT_PROGRAM
        self.events = queue.Queue()
        self._build_ui()
        self.after(100, self._drain_events)
        self.after(1000, self._refresh_state)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        if IMPORT_ERROR:
            self.log(f"Не удалось импортировать Dobot SDK: {IMPORT_ERROR}")
            self.log("Сначала запустите setup_windows.bat и убедитесь, что SDK установлен.")

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="IP робота:").pack(side="left")
        self.ip_var = tk.StringVar(value="192.168.5.1")
        ttk.Entry(top, textvariable=self.ip_var, width=16).pack(side="left", padx=(5, 12))
        ttk.Label(top, text="Скорость %:").pack(side="left")
        self.speed_var = tk.IntVar(value=5)
        ttk.Spinbox(top, from_=1, to=20, textvariable=self.speed_var, width=5).pack(side="left", padx=5)
        self.connect_btn = ttk.Button(top, text="Подключиться", command=self.connect_robot)
        self.connect_btn.pack(side="left", padx=8)
        self.disconnect_btn = ttk.Button(top, text="Отключиться", command=self.disconnect_robot, state="disabled")
        self.disconnect_btn.pack(side="left")
        self.status_var = tk.StringVar(value="Не подключено")
        ttk.Label(top, textvariable=self.status_var, foreground="#174a7e").pack(side="right")

        safety = ttk.LabelFrame(self, text="Сначала безопасность", padding=8)
        safety.pack(fill="x", padx=10, pady=(0, 8))
        ttk.Label(safety, text="Проверьте рабочую зону, нагрузку ES01, режим управления и доступность штатного аварийного стопа. Первые движения — без предмета и на малой скорости.", wraplength=980).pack(anchor="w")

        body = ttk.Frame(self, padding=(10, 0, 10, 8))
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        left.pack(side="left", fill="y", padx=(0, 10))
        conn = ttk.LabelFrame(left, text="Ручное обучение", padding=8)
        conn.pack(fill="x", pady=(0, 8))
        self.enable_btn = ttk.Button(conn, text="1. Включить питание суставов", command=self.enable_robot, state="disabled")
        self.enable_btn.pack(fill="x", pady=3)
        self.teach_btn = ttk.Button(conn, text="2. Начать ручное ведение", command=self.start_teach, state="disabled")
        self.teach_btn.pack(fill="x", pady=3)
        self.save_btn = ttk.Button(conn, text="Сохранить текущую позу…", command=self.save_point, state="disabled")
        self.save_btn.pack(fill="x", pady=3)
        self.stop_teach_btn = ttk.Button(conn, text="Завершить ручное ведение", command=self.stop_teach, state="disabled")
        self.stop_teach_btn.pack(fill="x", pady=3)
        ttk.Separator(conn).pack(fill="x", pady=7)
        self.vac_on_btn = ttk.Button(conn, text="Вакуум ВКЛ + записать событие", command=lambda: self.set_vacuum(True), state="disabled")
        self.vac_on_btn.pack(fill="x", pady=3)
        self.vac_off_btn = ttk.Button(conn, text="Вакуум ВЫКЛ + записать событие", command=lambda: self.set_vacuum(False), state="disabled")
        self.vac_off_btn.pack(fill="x", pady=3)

        files = ttk.LabelFrame(left, text="Программа", padding=8)
        files.pack(fill="x", pady=(0, 8))
        ttk.Button(files, text="Новая запись", command=self.new_program).pack(fill="x", pady=3)
        ttk.Button(files, text="Открыть JSON…", command=self.open_program).pack(fill="x", pady=3)
        ttk.Button(files, text="Сохранить JSON…", command=self.save_program_as).pack(fill="x", pady=3)
        ttk.Button(files, text="Обновить список", command=self.refresh_points).pack(fill="x", pady=3)
        ttk.Label(files, text="Файл:", wraplength=220).pack(anchor="w", pady=(5, 0))
        self.file_var = tk.StringVar(value=str(self.program_path))
        ttk.Label(files, textvariable=self.file_var, wraplength=220).pack(anchor="w")

        play = ttk.LabelFrame(left, text="Повторение", padding=8)
        play.pack(fill="x")
        self.play_btn = ttk.Button(play, text="Проверить точки и воспроизвести", command=self.play_program, state="disabled")
        self.play_btn.pack(fill="x", pady=3)
        self.reverse_btn = ttk.Button(play, text="Обратный порядок точек (тест)", command=self.reverse_program, state="disabled")
        self.reverse_btn.pack(fill="x", pady=3)
        ttk.Button(play, text="Остановить движение", command=self.stop_motion).pack(fill="x", pady=3)
        ttk.Label(play, text="Обратный порядок не является безопасным автоматическим возвратом предмета: вакуумные события требуют отдельной проверки.", wraplength=220).pack(anchor="w", pady=(6, 0))

        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)
        pose_frame = ttk.LabelFrame(right, text="Текущее состояние", padding=8)
        pose_frame.pack(fill="x", pady=(0, 8))
        self.joints_var = tk.StringVar(value="Суставные углы: —")
        self.pose_var = tk.StringVar(value="TCP: —")
        self.mode_var = tk.StringVar(value="Режим: —")
        ttk.Label(pose_frame, textvariable=self.mode_var).pack(anchor="w")
        ttk.Label(pose_frame, textvariable=self.joints_var).pack(anchor="w")
        ttk.Label(pose_frame, textvariable=self.pose_var).pack(anchor="w")
        table_frame = ttk.LabelFrame(right, text="Записанные точки", padding=6)
        table_frame.pack(fill="both", expand=True, pady=(0, 8))
        columns = ("n", "name", "vacuum", "joints")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", height=11)
        for col, title, width in (("n", "№", 45), ("name", "Имя точки", 180), ("vacuum", "Вакуум", 75), ("joints", "Углы суставов", 440)):
            self.table.heading(col, text=title)
            self.table.column(col, width=width, anchor="w")
        yscroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=yscroll.set)
        self.table.pack(side="left", fill="both", expand=True)
        yscroll.pack(side="right", fill="y")
        log_frame = ttk.LabelFrame(right, text="Журнал", padding=6)
        log_frame.pack(fill="both")
        self.log_box = tk.Text(log_frame, height=8, wrap="word", state="disabled")
        self.log_box.pack(fill="both", expand=True)
        self.refresh_points()

    def log(self, message):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{datetime.now().strftime('%H:%M:%S')}] {message}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _drain_events(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "log": self.log(data)
                elif kind == "done":
                    self.busy = False
                    self._set_controls()
                    if data: self.log(data)
                elif kind == "error":
                    self.busy = False
                    self._set_controls()
                    self.log("ОШИБКА: " + data)
                    messagebox.showerror("Ошибка", data)
        except queue.Empty:
            pass
        self.after(100, self._drain_events)

    def _task(self, label, fn):
        if self.busy:
            messagebox.showwarning("Занято", "Дождитесь завершения текущей операции.")
            return
        self.busy = True
        self._set_controls()
        self.log(label)
        def runner():
            try:
                result = fn()
                self.events.put(("done", result or f"Готово: {label}"))
            except Exception as exc:
                self.events.put(("error", f"{exc}\n\n{traceback.format_exc(limit=3)}"))
        threading.Thread(target=runner, daemon=True).start()

    def _set_controls(self):
        connected = self.arm is not None
        normal = "disabled" if self.busy else "normal"
        self.connect_btn.configure(state="disabled" if connected or self.busy else "normal")
        self.disconnect_btn.configure(state="normal" if connected and not self.busy else "disabled")
        self.enable_btn.configure(state=normal if connected else "disabled")
        self.teach_btn.configure(state=normal if connected and not self.teaching else "disabled")
        self.save_btn.configure(state=normal if connected and self.teaching else "disabled")
        self.stop_teach_btn.configure(state=normal if connected and self.teaching else "disabled")
        self.vac_on_btn.configure(state=normal if connected and self.teaching else "disabled")
        self.vac_off_btn.configure(state=normal if connected and self.teaching else "disabled")
        self.play_btn.configure(state=normal if connected and len(self.points) >= 2 and not self.teaching else "disabled")
        self.reverse_btn.configure(state=normal if connected and len(self.points) >= 2 and not self.teaching else "disabled")

    def connect_robot(self):
        if IMPORT_ERROR:
            messagebox.showerror("SDK не установлен", f"{IMPORT_ERROR}\nЗапустите setup_windows.bat.")
            return
        ip = self.ip_var.get().strip()
        def op():
            arm = ArmClient(ip=ip)
            arm.connect()
            self.arm = arm
            self.events.put(("log", f"Подключено к {ip}; режим {arm.mode_name()}"))
            self.after(0, self._set_controls)
            return "Соединение установлено. Перед Enable проверьте рабочую зону."
        self._task("Подключение к роботу…", op)

    def disconnect_robot(self):
        if self.teaching:
            messagebox.showwarning("Ручное ведение", "Сначала нажмите «Завершить ручное ведение».")
            return
        def op():
            arm = self.arm
            self.arm = None
            if arm: arm.close()
            self.events.put(("log", "Соединение закрыто."))
            self.after(0, self._set_controls)
            return "Отключено."
        self._task("Отключение…", op)

    def enable_robot(self):
        if not messagebox.askyesno("Включить питание", "Суставы будут запитаны. Убедитесь, что зона свободна и рука поддерживается при необходимости."):
            return
        self._task("Включение суставов…", lambda: (self.arm.enable(load_kg=ES01_MASS_KG), self.arm.speed(int(self.speed_var.get())), "Суставы запитаны." )[2])

    def start_teach(self):
        if not messagebox.askyesno("Ручное ведение", "При включении рука может просесть под весом. Поддерживайте манипулятор и уберите предметы. Начать?"):
            return
        def op():
            self.arm.gripper(False)
            self.vacuum = False
            self.arm.enable(load_kg=ES01_MASS_KG)
            self.arm.speed(int(self.speed_var.get()))
            self.arm.drag(True)
            self.teaching = True
            self.after(0, self._set_controls)
            return "Ручное ведение включено. Физически переместите руку и сохраняйте именованные точки."
        self._task("Включение ручного ведения…", op)

    def _capture(self, name, vacuum):
        return {"name": name, "joints": [round(float(x), 5) for x in self.arm.joints()], "pose": [round(float(x), 4) for x in self.arm.pose()], "vacuum": bool(vacuum)}

    def save_point(self):
        name = simpledialog.askstring("Сохранить точку", "Имя точки (например pick_approach):", parent=self)
        if name is None: return
        name = name.strip() or f"point_{len(self.points)+1:03d}"
        def op():
            self.points.append(self._capture(name, self.vacuum))
            self.after(0, self.refresh_points)
            return f"Сохранена точка {name}."
        self._task("Чтение текущей позы…", op)

    def set_vacuum(self, on):
        if not messagebox.askyesno("Вакуум", f"{'Включить' if on else 'Выключить'} вакуум ES01? Убедитесь, что это безопасно."):
            return
        name = simpledialog.askstring("Событие вакуума", "Имя события:", initialvalue=("pick_grip" if on else "place_release"), parent=self)
        if name is None: return
        def op():
            self.arm.gripper(on)
            self.vacuum = on
            self.points.append(self._capture(name.strip() or f"vacuum_{'on' if on else 'off'}", on))
            self.after(0, self.refresh_points)
            return f"Вакуум {'ВКЛ' if on else 'ВЫКЛ'}; событие записано в текущей позе."
        self._task("Управление вакуумом…", op)

    def stop_teach(self):
        def op():
            self.arm.drag(False)
            self.teaching = False
            self.after(0, self._set_controls)
            return "Ручное ведение выключено. Сохраните программу в JSON."
        self._task("Завершение обучения…", op)

    def new_program(self):
        if self.teaching:
            messagebox.showwarning("Обучение активно", "Сначала завершите ручное ведение.")
            return
        if messagebox.askyesno("Новая программа", "Очистить текущий список точек?"):
            self.points = []
            self.refresh_points()
            self._set_controls()

    def refresh_points(self):
        for item in self.table.get_children(): self.table.delete(item)
        for i, p in enumerate(self.points, 1):
            self.table.insert("", "end", values=(i, p["name"], "ВКЛ" if p["vacuum"] else "ВЫКЛ", ", ".join(f"{v:.2f}" for v in p["joints"])))
        if hasattr(self, "play_btn"): self._set_controls()

    def save_program_as(self):
        path = filedialog.asksaveasfilename(title="Сохранить программу", defaultextension=".json", initialfile=self.program_path.name, filetypes=[("JSON", "*.json")])
        if not path: return
        try:
            pts = validate_points([dict(p) for p in self.points])
            data = {"schema_version": SCHEMA_VERSION, "robot": "Dobot Magician E6", "tool": "DOBOT ES01", "coordinate_source": "q_actual + tool_vector_actual from feedback port 30004", "created_at": datetime.now(timezone.utc).isoformat(), "motion_type": "joint", "points": pts}
            Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            self.program_path = Path(path)
            self.file_var.set(str(self.program_path))
            self.log(f"Сохранено {len(pts)} точек в {path}")
        except Exception as exc: messagebox.showerror("Не удалось сохранить", str(exc))

    def open_program(self):
        path = filedialog.askopenfilename(title="Открыть программу", filetypes=[("JSON", "*.json")])
        if not path: return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if data.get("schema_version") != SCHEMA_VERSION: raise ValueError("Неподдерживаемая версия файла.")
            self.points = validate_points(data.get("points"))
            self.program_path = Path(path)
            self.file_var.set(str(self.program_path))
            self.refresh_points()
            self.log(f"Загружено {len(self.points)} точек из {path}")
        except Exception as exc: messagebox.showerror("Не удалось открыть программу", str(exc))

    def _preflight(self, points):
        if len(points) < 2: raise ValueError("Нужно минимум две точки.")
        for p in points:
            if not self.arm.reachable(p["pose"]):
                raise ArmError(f"Точка {p['name']} не проходит проверку достижимости.")
        return True

    def play_program(self):
        try: pts = validate_points([dict(p) for p in self.points])
        except Exception as exc:
            messagebox.showerror("Ошибка программы", str(exc)); return
        if not messagebox.askyesno("Подтвердить движение", "Робот автоматически пройдёт все записанные точки.\n\nУберите лишние предметы, проверьте траекторию, скорость и аварийный стоп. Первый запуск выполняйте без предмета. Продолжить?"):
            return
        def op():
            self._preflight(pts)
            self.arm.enable(load_kg=ES01_MASS_KG)
            self.arm.speed(int(self.speed_var.get()))
            previous = self.arm.gripper_state()
            for i, p in enumerate(pts, 1):
                if self.arm.mode() == 9 or self.arm.errors(): raise ArmError(f"Авария перед точкой {p['name']}: {self.arm.errors()}")
                self.events.put(("log", f"[{i}/{len(pts)}] → {p['name']}"))
                self.arm.move_joints(p["joints"])
                if p["vacuum"] != previous:
                    self.arm.gripper(p["vacuum"])
                    previous = p["vacuum"]
            return "Воспроизведение завершено."
        self._task("Проверка достижимости и подготовка воспроизведения…", op)

    def reverse_program(self):
        messagebox.showwarning("Обратный порядок", "Эта функция только тестовая: обратный порядок суставных точек НЕ реализует безопасный возврат предмета. События вакуума при обратном движении могут быть неверными. Используйте её только без предмета, после проверки траектории.")
        if not messagebox.askyesno("Подтвердить обратный тест", "Выполнить точки в обратном порядке без предмета?"):
            return
        pts = list(reversed([dict(p) for p in self.points]))
        def op():
            self._preflight(pts)
            self.arm.enable(load_kg=ES01_MASS_KG)
            self.arm.speed(int(self.speed_var.get()))
            self.arm.gripper(False)
            for i, p in enumerate(pts, 1):
                if self.arm.mode() == 9 or self.arm.errors(): raise ArmError(f"Авария перед точкой {p['name']}: {self.arm.errors()}")
                self.events.put(("log", f"[обратный {i}/{len(pts)}] → {p['name']}"))
                self.arm.move_joints(p["joints"])
            return "Обратный тест точек завершён; вакуум оставлен выключенным."
        self._task("Обратный тест без предмета…", op)

    def stop_motion(self):
        if self.arm is None:
            self.log("Робот не подключён."); return
        try:
            self.arm.stop()
            self.log("Отправлена команда Stop. При опасности используйте штатный аварийный стоп.")
        except Exception as exc: messagebox.showerror("Stop не выполнен", str(exc))

    def _refresh_state(self):
        if self.arm is not None and not self.busy:
            try:
                j = self.arm.joints(); p = self.arm.pose(); mode = self.arm.mode_name()
                self.joints_var.set("Суставы: " + ", ".join(f"{x:.1f}°" for x in j))
                self.pose_var.set("TCP: " + ", ".join(f"{x:.1f}" for x in p))
                self.mode_var.set(f"Режим: {mode} | вакуум: {'ВКЛ' if self.vacuum else 'ВЫКЛ'}")
                self.status_var.set("Подключено")
            except Exception as exc:
                self.status_var.set("Ошибка чтения")
        else:
            self.status_var.set("Подключено" if self.arm is not None else "Не подключено")
        self.after(1000, self._refresh_state)

    def _on_close(self):
        if self.busy:
            if not messagebox.askyesno("Операция выполняется", "Закрыть окно во время операции? Это не заменяет аварийный стоп."):
                return
        if self.teaching and self.arm:
            try: self.arm.drag(False)
            except Exception: pass
        if self.arm:
            try: self.arm.close()
            except Exception: pass
        self.destroy()


if __name__ == "__main__":
    RoboHandApp().mainloop()
