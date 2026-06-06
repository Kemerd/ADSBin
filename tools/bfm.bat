@echo off
REM ===========================================================================
REM  ADSBin :: one-shot build -> flash -> monitor (the main test-loop button)
REM  Usage:  tools\bfm.bat [COMx]
REM          tools\bfm.bat            (auto-detect the serial port)
REM  Requires an active ESP-IDF environment (run from an "ESP-IDF" prompt so
REM  idf.py is on PATH). Ctrl-] exits the monitor.
REM ===========================================================================
chcp 65001 >nul
setlocal
set PORT=%~1
if "%PORT%"=="" (
    echo [ADSBin] build + flash + monitor ^(auto-detect port^) ...
    idf.py build flash monitor
) else (
    echo [ADSBin] build + flash + monitor on %PORT% ...
    idf.py -p %PORT% build flash monitor
)
endlocal
