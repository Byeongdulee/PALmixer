@echo off
rem ======================================================================
rem  Start PALmixer: the control server and the operator GUI, one window each.
rem
rem      run_palmixer.cmd              drives the real robot, motor, and pumps
rem      run_palmixer.cmd --simulate   no robot/motor/pump I/O; for GUI/dev work
rem
rem  Everything you pass is handed to the server; the GUI takes no arguments.
rem  Runs in the aps12robot conda environment. Override with:
rem      set PALMIXER_PYTHON=C:\path\to\python.exe
rem
rem  The two pump dashboards (apssector12_pump_control: pump_dashboard.py on
rem  5555, flowcell_dashboard.py on 5556) are separate programs and are NOT
rem  started here. Without them the Pumps panel reports both as unreachable
rem  and every pump step fails -- start them first.
rem ======================================================================
setlocal

rem palmixer is not pip-installed into the environment; it is imported from
rem this folder. So every check below, and both windows, run from here -- the
rem two `start` commands inherit this directory.
cd /d "%~dp0"

if not defined PALMIXER_PYTHON (
    set "PALMIXER_PYTHON=%USERPROFILE%\Anaconda3\envs\aps12robot\python.exe"
)

rem -- 1. the interpreter exists and can import the package -----------------
if not exist "%PALMIXER_PYTHON%" (
    echo.
    echo No interpreter at:
    echo     %PALMIXER_PYTHON%
    echo Create the aps12robot environment, or point PALMIXER_PYTHON at the
    echo python.exe you want:
    echo     set PALMIXER_PYTHON=C:\path\to\python.exe
    echo.
    pause
    exit /b 1
)

"%PALMIXER_PYTHON%" -c "import palmixer" 2>nul
if errorlevel 1 (
    echo.
    echo "%PALMIXER_PYTHON%" cannot import palmixer from:
    echo     %CD%
    echo Run this script from a full checkout -- it imports the package from
    echo its own folder rather than from site-packages.
    echo.
    pause
    exit /b 1
)

rem -- 2. the pieces each window needs -------------------------------------
rem Checked here because both fail late otherwise: the GUI aborts on the PyQt5
rem import in a window that closes, and pyepics is reached only when the
rem carousel motor is first moved -- i.e. part-way through a real make_sample.
"%PALMIXER_PYTHON%" -c "import importlib.util, sys; missing = [m for m in ('zmq', 'paho.mqtt', 'PyQt5', 'epics') if not importlib.util.find_spec(m.split('.')[0])]; print('missing: ' + ', '.join(missing)) if missing else None; sys.exit(1 if missing else 0)"
if errorlevel 1 (
    echo.
    echo Install the missing packages into that environment:
    echo     "%PALMIXER_PYTHON%" -m pip install pyzmq paho-mqtt PyQt5 pyepics
    echo.
    pause
    exit /b 1
)

rem -- 3. the robot driver, unless this is a simulate run -------------------
rem Only the server imports robot12idb, and only when driving real hardware.
rem It raises a clear ImportError naming the path it searched -- but in a
rem window that closes, so the same check runs here first.
rem
rem Plain batch substring matching rather than find/findstr: Git for Windows
rem puts GNU versions of both on PATH and those parse these arguments
rem differently. The leading x matters -- "set PM_ARGS=" with no arguments
rem leaves the variable undefined, and %PM_ARGS:--simulate=% on an undefined
rem variable does not expand, so the comparison below would never match.
set "PM_ARGS=x%*"
set "PM_NOSIM=%PM_ARGS:--simulate=%"
if "%PM_ARGS%"=="%PM_NOSIM%" (
    "%PALMIXER_PYTHON%" -c "import sys; from palmixer import config; p = config.get_config()['robot'].get('ur12idb_path'); sys.path.insert(0, p) if p else None; import robot12idb, camera_tools; print('UR_12idb: ' + str(p))"
    if errorlevel 1 (
        echo.
        echo The server cannot import the UR_12idb robot driver -- see the error
        echo above. Point PALMIXER_UR12IDB_PATH at your checkout:
        echo     set PALMIXER_UR12IDB_PATH=C:\path\to\UR_12idb
        echo or start without hardware:
        echo     run_palmixer.cmd --simulate
        echo.
        pause
        exit /b 1
    )
)

rem -- 4. is an older server still holding the port? ------------------------
rem The server reports this itself and exits, but again in a window that
rem closes before anyone reads it. The port always comes from the config --
rem unlike PALsystem's server, this one takes no --port argument.
"%PALMIXER_PYTHON%" -c "import socket, sys; from palmixer import config; port = config.get_section('zmq').get('port', 9880); sock = socket.socket(); sock.settimeout(1.0); busy = sock.connect_ex(('127.0.0.1', port)) == 0; sock.close(); print('PALmixer port ' + str(port) + (' : ALREADY IN USE' if busy else ' : free')); sys.exit(1 if busy else 0)"
if errorlevel 1 (
    echo.
    echo A PALmixer server is already running on that port. Stop it first
    echo -- Ctrl+C in its window, or close it -- then run this again.
    echo.
    pause
    exit /b 1
)

rem -- 5. the server -------------------------------------------------------
rem Wrapped in "cmd /c ... || pause" so a startup failure stays on screen.
rem A clean Ctrl+C exits 0, so the window still closes normally on shutdown.
echo Starting the PALmixer server...
start "PALmixer server" cmd /c ""%PALMIXER_PYTHON%" -m palmixer.server %* || pause"

rem On a real run the server connects to the UR3 before it starts answering,
rem which takes a few seconds -- so give it a head start before the GUI's
rem first poll goes out, or the GUI opens showing a dead server. ping rather
rem than timeout.exe: timeout refuses to run when stdin is redirected, and
rem full paths because Git for Windows ships GNU versions of both.
"%SystemRoot%\System32\ping.exe" -n 6 127.0.0.1 >nul

rem -- 6. the GUI ----------------------------------------------------------
echo Starting the PALmixer GUI...
start "PALmixer GUI" cmd /c ""%PALMIXER_PYTHON%" -m palmixer.gui.app || pause"

echo.
echo Both started. Close their windows to stop them.
endlocal
