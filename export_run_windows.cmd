@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
if exist "host\.venv_gui\Scripts\python.exe" (
 "host\.venv_gui\Scripts\python.exe" tools\export_run.py %*
) else (
 where py >nul 2>nul
 if errorlevel 1 (python tools\export_run.py %*) else (py -3 tools\export_run.py %*)
)
set "RESULT=%errorlevel%"
pause
exit /b %RESULT%
