@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
where py >nul 2>nul
if errorlevel 1 (python tools\environment_info.py) else (py -3 tools\environment_info.py)
pause
