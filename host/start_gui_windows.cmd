@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
echo J280 Studio R43 - FPGA build 4A070257 - Source and STA qualification checked by GUI
call setup_gui_windows.cmd
if errorlevel 1 goto failed
".venv_gui\Scripts\python.exe" -u ..\target_ui_fix\launch_target.py %*
if errorlevel 1 goto failed
exit /b 0
:failed
echo.
echo Launch failed. Keep this window open and copy the error above.
pause
exit /b 1
