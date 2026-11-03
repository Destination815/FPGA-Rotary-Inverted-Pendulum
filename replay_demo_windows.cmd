@echo off
setlocal
call "%~dp0start_R43_windows.cmd" --replay "%~dp0examples\demo\run.csv"
exit /b %errorlevel%
