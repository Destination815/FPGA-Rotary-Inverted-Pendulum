@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
if exist ".venv_gui\Scripts\python.exe" goto existing
py -3.12 -c "import sys" >nul 2>&1
if not errorlevel 1 goto python312
py -3 -c "import sys; assert (3,10) <= sys.version_info[:2] < (3,14)" >nul 2>&1
if not errorlevel 1 goto launcher
python -c "import sys; assert (3,10) <= sys.version_info[:2] < (3,14)" >nul 2>&1
if not errorlevel 1 goto pythonpath
echo Python 3.10-3.13 was not found. Install 64-bit Python 3.12:
echo https://www.python.org/downloads/windows/
echo Enable pip and the Python launcher, then try again.
exit /b 1
:existing
".venv_gui\Scripts\python.exe" prepare_gui.py
exit /b %errorlevel%
:python312
py -3.12 prepare_gui.py
exit /b %errorlevel%
:launcher
py -3 prepare_gui.py
exit /b %errorlevel%
:pythonpath
python prepare_gui.py
exit /b %errorlevel%
