@echo off
setlocal

set "ROOT=%~dp0"
set "LAUNCHER=%ROOT%Launch-CARLA-Project.ps1"

if not exist "%LAUNCHER%" (
    echo CARLA project launcher was not found:
    echo %LAUNCHER%
    pause
    exit /b 1
)

powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%LAUNCHER%" %*
set "EXITCODE=%ERRORLEVEL%"

echo.
if not "%EXITCODE%"=="0" (
    echo CARLA project launcher exited with code %EXITCODE%.
) else (
    echo CARLA project launcher finished successfully.
)
pause
exit /b %EXITCODE%
