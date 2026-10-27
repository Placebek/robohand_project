@echo off
setlocal
cd /d "%~dp0\.."
echo === RoboHand Windows setup ===
where py >nul 2>nul
if errorlevel 1 (
  echo Python 3.10-3.12 was not found. Install Python from python.org and enable Add Python to PATH.
  pause
  exit /b 1
)
if not exist .venv\Scripts\python.exe py -3 -m venv .venv
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip setuptools wheel
if not exist vendor\dobot_tcpip_v4 (
  where git >nul 2>nul
  if errorlevel 1 (
    echo Git is required to download the official Dobot TCP/IP SDK. Install Git for Windows and retry.
    pause
    exit /b 1
  )
  git clone --depth 1 https://github.com/Dobot-Arm/TCP-IP-Python-V4.git vendor\dobot_tcpip_v4
  if errorlevel 1 goto :fail
)
python -m pip install -e vendor\dobot_tcpip_v4
if errorlevel 1 goto :fail
python -c "from dobot_sdk import DobotRobot; print('Dobot SDK import OK')"
if errorlevel 1 goto :fail
echo Setup completed. Run windows_app\run_gui.bat
pause
exit /b 0
:fail
echo Setup failed. Copy the full error text and send it for troubleshooting.
pause
exit /b 1
