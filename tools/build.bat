@echo off
REM ===========================================================================
REM  ADSBin :: build only (no flash). Fast inner loop for compile checks.
REM ===========================================================================
chcp 65001 >nul
echo [ADSBin] building ...
idf.py build
