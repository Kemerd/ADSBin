@echo off
REM ===========================================================================
REM  ADSBin :: flash + monitor (no rebuild). Assumes a current build/.
REM  Usage:  tools\flash.bat [COMx]   (omit COMx to auto-detect)
REM ===========================================================================
chcp 65001 >nul
setlocal
set PORT=%~1
if "%PORT%"=="" (
    idf.py flash monitor
) else (
    idf.py -p %PORT% flash monitor
)
endlocal
