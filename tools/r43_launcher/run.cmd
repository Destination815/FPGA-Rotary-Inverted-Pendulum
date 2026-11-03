@echo off
setlocal EnableExtensions DisableDelayedExpansion
chcp 65001 >nul
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
if not exist "tools\compile_r40_once.py" goto project_missing
if not exist "source_manifest.json" goto project_missing
if not exist "host\.venv_gui\Scripts\python.exe" goto try_py
echo Checking project Python...
>> "R43_startup.log" echo Checking project Python...
"host\.venv_gui\Scripts\python.exe" -c "import sys; print(sys.version); sys.exit(sys.version_info[:2] < (3,9))" >> "R43_startup.log" 2>&1
if errorlevel 1 goto try_py
"host\.venv_gui\Scripts\python.exe" -B -u "tools\r43_launcher\launch.py" %*
exit /b %errorlevel%
:try_py
echo Checking py.exe...
>> "R43_startup.log" echo Checking py.exe...
py.exe -3 -c "import sys; print(sys.version); sys.exit(sys.version_info[:2] < (3,9))" >> "R43_startup.log" 2>&1
if errorlevel 1 goto try_python
py.exe -3 -B -u "tools\r43_launcher\launch.py" %*
exit /b %errorlevel%
:try_python
echo Checking python.exe...
>> "R43_startup.log" echo Checking python.exe...
python.exe -c "import sys; print(sys.version); sys.exit(sys.version_info[:2] < (3,9))" >> "R43_startup.log" 2>&1
if errorlevel 1 goto python_missing
python.exe -B -u "tools\r43_launcher\launch.py" %*
exit /b %errorlevel%
:python_missing
echo ERROR: No working Python 3.9 or newer was found.
echo Install Python 3.12 with the Python launcher, then run this CMD again.
echo Python download: https://www.python.org/downloads/windows/
>> "R43_startup.log" echo ERROR: No working Python 3.9 or newer was found.
exit /b 2
:project_missing
echo ERROR: Extract this patch into the R43 project folder beside source_manifest.json.
>> "R43_startup.log" echo ERROR: Original R43 project files were not found beside the launcher.
exit /b 2
