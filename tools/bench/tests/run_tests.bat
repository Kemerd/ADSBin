@echo off
REM ===========================================================================
REM  run_tests.bat - Run the ADSBin host-only unit tests (UTF-8 / console safe).
REM
REM  These tests prove the PURE algorithms (CPR, GDL90 framing/CRC, sink_debug
REM  token format) on a PC, with no firmware and no hardware. They are the host
REM  twin of the device contracts in components/ and tools/bench/WIRE_CONTRACT.md.
REM
REM  Usage:
REM     run_tests.bat            run the whole suite (quiet)
REM     run_tests.bat -v         run verbose (one line per test)
REM ===========================================================================

REM --- Make the console UTF-8 so any non-ASCII output renders cleanly. -------
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1

REM --- Always run from this script's own directory so imports resolve. -------
pushd "%~dp0"

REM --- Locate a Python interpreter: prefer the launcher, fall back to python. -
where py >nul 2>nul
if %ERRORLEVEL%==0 (
    set "PY=py -3"
) else (
    set "PY=python"
)

echo === ADSBin host unit tests ===
echo Using interpreter: %PY%
echo.

REM --- Discover and run every test_*.py in this folder. ----------------------
%PY% -m unittest discover -s . -p "test_*.py" %*
set "RC=%ERRORLEVEL%"

echo.
if "%RC%"=="0" (
    echo === ALL TESTS PASSED ===
) else (
    echo === TESTS FAILED ^(exit %RC%^) ===
)

popd
exit /b %RC%
