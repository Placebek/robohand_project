@echo off
cd /d "%~dp0\.."
if not exist .venv\Scripts\python.exe (
 echo Run windows_app\setup_windows.bat first.
 pause
 exit /b 1
)
.venv\Scripts\python.exe windows_app\robohand_gui.py
if errorlevel 1 pause
