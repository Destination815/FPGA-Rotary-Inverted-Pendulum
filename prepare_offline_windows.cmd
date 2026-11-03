@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
py -3.12 --version >nul 2>nul
if errorlevel 1 goto use_python
py -3.12 tools\offline_dependencies.py %*
goto finish
:use_python
python tools\offline_dependencies.py %*
:finish
set "RESULT=%errorlevel%"
pause
exit /b %RESULT%
