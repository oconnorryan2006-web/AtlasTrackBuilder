@echo off
cd /d "%~dp0"
if exist "dist\ATLAS World Studio.exe" (
    start "" "dist\ATLAS World Studio.exe"
    exit /b 0
)
echo Build the application first by running build_windows.bat.
pause
exit /b 1
