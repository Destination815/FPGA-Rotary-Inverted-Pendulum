@echo off
setlocal EnableExtensions DisableDelayedExpansion
title R43 compiler
pushd "%~dp0"
if errorlevel 1 goto folder_failed
echo R43 / 4A070257 - compiler launcher
echo Project: "%CD%"
echo Startup log: "%CD%\R43_startup.log"
> "R43_startup.log" echo R43 compiler launcher started.
if errorlevel 1 goto log_failed
if not exist "tools\r43_launcher\run.cmd" goto launcher_missing
"%SystemRoot%\System32\cmd.exe" /d /e:on /v:off /s /c ""tools\r43_launcher\run.cmd" %*"
set "J280_R43_EXIT=%errorlevel%"
>> "R43_startup.log" echo Launcher exit code: %J280_R43_EXIT%
echo.
echo Launcher exit code: %J280_R43_EXIT%
echo Return the *_RETURN.zip shown above.
echo If no RETURN.zip was created, send R43_startup.log from this project.
goto finish
:launcher_missing
echo ERROR: Extract the complete launcher patch into the R43 project folder.
>> "R43_startup.log" echo ERROR: tools\r43_launcher\run.cmd is missing.
set "J280_R43_EXIT=2"
goto finish
:log_failed
echo ERROR: Cannot write R43_startup.log in this project folder.
set "J280_R43_EXIT=2"
goto finish
:folder_failed
echo ERROR: Cannot open the folder containing this CMD file.
pause
exit /b 2
:finish
echo.
pause
popd
exit /b %J280_R43_EXIT%
