@echo off
REM ===========================================================================
REM  ADSBin :: serial monitor only (Ctrl-] to exit).
REM  Usage:  tools\monitor.bat [COMx]   (omit COMx to auto-detect)
REM ===========================================================================
chcp 65001 >nul
setlocal
set PORT=%~1
if "%PORT%"=="" (
    idf.py monitor
) else (
    idf.py -p %PORT% monitor
)
endlocal
