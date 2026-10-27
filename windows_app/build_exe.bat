@echo off
setlocal
cd /d "%~dp0\.."
if not exist .venv\Scripts\python.exe (
 echo Run windows_app\setup_windows.bat first.
 pause
 exit /b 1
)
call .venv\Scripts\activate.bat
python -m pip install --upgrade pyinstaller
python -m PyInstaller --noconfirm --clean --onedir --windowed --name RoboHandTeachRepeat --paths . --collect-all dobot_sdk windows_app\robohand_gui.py
if errorlevel 1 goto :fail
echo Build completed: dist\RoboHandTeachRepeat\RoboHandTeachRepeat.exe
echo Keep the whole dist\RoboHandTeachRepeat folder together; this is an onedir build.
pause
exit /b 0
:fail
echo Build failed. Send the error output.
pause
exit /b 1
