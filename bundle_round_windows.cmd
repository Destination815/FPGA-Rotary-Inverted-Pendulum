@echo off
setlocal
chcp 65001 >nul
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
cd /d "%~dp0"
if exist "host\.venv_gui\Scripts\python.exe" (
  "host\.venv_gui\Scripts\python.exe" tools\bundle_round.py %*
) else (
  where py >nul 2>nul
  if errorlevel 1 (
    python tools\bundle_round.py %*
  ) else (
    py -3 tools\bundle_round.py %*
  )
)
set "R33_TOOL_EXIT=%errorlevel%"
pause
exit /b %R33_TOOL_EXIT%
