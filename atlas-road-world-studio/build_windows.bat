@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "STEP=Verify required application files"
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"

if not exist "src\launch.py" goto missing
if not exist "src\server.py" goto missing
if not exist "src\static\index.html" goto missing
if not exist "src\static\brand\atlas.ico" goto missing
if not exist "src\road_network_v3.py" goto missing
if not exist "src\buildings.py" goto missing
if not exist "src\road_signs.py" goto missing
if not exist "src\mesh_validation.py" goto missing
if not exist "tests\test_export_pipeline.py" goto missing
if not exist "requirements.txt" goto missing

set "STEP=Locate Python"
rem Prefer the Python 3.14 installation that has PyInstaller.
set "PYEXE=%LOCALAPPDATA%\Python\pythoncore-3.14-64\python.exe"
if exist "%PYEXE%" goto python_ready
set "PYEXE="
for /f "delims=" %%P in ('py -3.14 -c "import sys; print(sys.executable)" 2^>nul') do set "PYEXE=%%P"
if defined PYEXE goto python_ready
for /f "delims=" %%P in ('py -3 -c "import sys; print(sys.executable)" 2^>nul') do set "PYEXE=%%P"
if defined PYEXE goto python_ready
for /f "delims=" %%P in ('python -c "import sys; print(sys.executable)" 2^>nul') do set "PYEXE=%%P"
if not defined PYEXE goto no_python

:python_ready
echo Using Python: "%PYEXE%"
set "STEP=Verify pip"
"%PYEXE%" -m pip --version >nul 2>&1
if errorlevel 1 goto no_pip
set "STEP=Install project dependencies"
"%PYEXE%" -m pip install -r requirements.txt pyinstaller
if errorlevel 1 goto fail

set "STEP=Run generation tests"
"%PYEXE%" -m unittest discover -s tests -v
if errorlevel 1 goto fail

rem The Windows GUI bootloader is required for a windowed EXE.
set "STEP=Verify PyInstaller Windows GUI bootloader"
set "BOOTLOADER="
"%PYEXE%" -c "import PyInstaller,pathlib; print(pathlib.Path(PyInstaller.__file__).parent / 'bootloader' / 'Windows-64bit-intel' / 'runw.exe')" > "%TEMP%\atlas_bootloader_path.txt"
if errorlevel 1 goto fail
set /p "BOOTLOADER=" < "%TEMP%\atlas_bootloader_path.txt"
if not exist "%BOOTLOADER%" goto missing_bootloader

rem Avoid reusing a stale spec inside build\ with broken relative paths.
if exist "build\ATLAS World Studio.spec" del /q "build\ATLAS World Studio.spec"
if not exist "dist" mkdir "dist"
echo.
echo Building ATLAS World Studio...
set "STEP=PyInstaller build"
tasklist /fi "IMAGENAME eq ATLAS World Studio.exe" /nh 2>nul | find /i "ATLAS World Studio.exe" >nul
if not errorlevel 1 goto app_running
"%PYEXE%" -m PyInstaller --noconfirm --clean --onefile --windowed --name "ATLAS World Studio" --icon "%CD%\src\static\brand\atlas.ico" --distpath "%CD%\dist" --workpath "%CD%\build" --specpath "%CD%\build" --paths "%CD%\src" --add-data "%CD%\src\server.py;." --add-data "%CD%\src\static;static" --add-data "%CD%\src\road_network_v3.py;." --hidden-import road_network_v3 --hidden-import server --hidden-import mesh_validation --hidden-import buildings --hidden-import road_signs --hidden-import sign_models --hidden-import roadside_hedges --collect-all shapely --collect-all PySide6.QtWebEngineCore --collect-all PySide6.QtWebEngineWidgets --collect-all PySide6.QtWebEngineQuick --collect-all PIL --hidden-import PIL.Image --hidden-import road_network_v3 --collect-data pyproj "%CD%\src\launch.py"
if errorlevel 1 goto fail
if not exist "dist\ATLAS World Studio.exe" goto fail

echo.
echo BUILD SUCCESSFUL
echo EXE: "%CD%\dist\ATLAS World Studio.exe"
echo Run this new EXE from dist, not an older copy in the project folder.
explorer "%CD%\dist"
pause
exit /b 0

:missing_bootloader
echo ERROR: PyInstaller runw.exe is missing. The Windows GUI bootloader must be installed correctly.
goto fail
:app_running
echo ERROR: Close ATLAS World Studio before rebuilding. Existing EXE was not overwritten.
set "STEP=Check for a running ATLAS application"
goto fail
:missing
echo ERROR: Required application module, requirements.txt, or static\index.html is missing.
goto fail
:no_python
echo ERROR: Python not found.
goto fail
:no_pip
echo ERROR: pip is unavailable for the selected Python.
goto fail
:fail
echo.
echo BUILD FAILED at step: %STEP%
echo Check the exact command and its output immediately above.
pause
exit /b 1
