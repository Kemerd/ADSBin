@echo off
REM ===========================================================================
REM  qc.bat - ADSBin manufacturing QC launcher (double-click to test a unit).
REM
REM  Plug a freshly-built ADSBin unit into USB, double-click this file, and the
REM  GUI auto-detects the device, runs a fast PASS/FAIL sweep (USB enumerate,
REM  dongle streaming, decode path via +INJECT, GDL90 CRC) and then keeps a live
REM  status panel updating (SDR / RF / aircraft / temperature / GPS / health).
REM
REM  Usage:
REM     qc.bat              auto-detect the unit by USB VID:PID
REM     qc.bat COM7         force a specific serial port
REM
REM  Requires Python 3 (the py launcher or python on PATH) + pyserial. If pyserial
REM  is missing this script installs it on first run. Tkinter ships with CPython.
REM ===========================================================================

REM --- Make the console UTF-8 so any non-ASCII output renders cleanly. --------
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1

REM --- Run from the bench folder so the qc_gui imports resolve. ---------------
pushd "%~dp0bench"

REM --- Locate a Python interpreter: prefer the launcher, fall back to python. -
where py >nul 2>nul
if %ERRORLEVEL%==0 (
    set "PY=py -3"
) else (
    set "PY=python"
)

REM --- Ensure pyserial is present (the only third-party dependency). ----------
%PY% -c "import serial" >nul 2>nul
if not %ERRORLEVEL%==0 (
    echo [setup] Installing pyserial ^(one-time^)...
    %PY% -m pip install --quiet pyserial
    if not %ERRORLEVEL%==0 (
        echo [error] Could not install pyserial. Install it manually:
        echo         %PY% -m pip install pyserial
        popd
        pause
        exit /b 1
    )
)

REM --- Launch the QC dashboard, passing through any explicit COM port. --------
echo Launching ADSBin Unit QC...
%PY% qc_gui.py %*
set "RC=%ERRORLEVEL%"

popd
REM --- On a non-zero exit (e.g. a crash), hold the window so the operator can
REM     read the error rather than have it vanish on a double-click.
if not "%RC%"=="0" (
    echo.
    echo qc_gui exited with code %RC%.
    pause
)
exit /b %RC%
