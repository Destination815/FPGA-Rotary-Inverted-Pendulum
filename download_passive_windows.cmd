@echo off
setlocal
chcp 65001 >nul
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
cd /d "%~dp0"
if exist "host\.venv_gui\Scripts\python.exe" (
  "host\.venv_gui\Scripts\python.exe" host\download_passive.py %*
) else (
  where py >nul 2>nul
  if errorlevel 1 (
    python host\download_passive.py %*
  ) else (
    py -3 host\download_passive.py %*
  )
)
set "R31_TOOL_EXIT=%errorlevel%"
pause
exit /b %R31_TOOL_EXIT%
