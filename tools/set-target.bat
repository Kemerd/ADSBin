@echo off
REM ===========================================================================
REM  ADSBin :: one-time target set. Run ONCE after cloning (or after deleting
REM  the build/ dir). Sets the chip to ESP32-P4 so all later builds target it.
REM ===========================================================================
chcp 65001 >nul
echo [ADSBin] setting target esp32p4 ...
idf.py set-target esp32p4
