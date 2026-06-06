# ============================================================================
#  ADSBin - top-level project build file
# ----------------------------------------------------------------------------
#  ADSBin ("ADS-B In") is a receive-only 1090 MHz ADS-B box built on the
#  Espressif ESP32-P4. This file is the ESP-IDF project entry point: it pulls in
#  the IDF build machinery and declares the project name. Component discovery is
#  automatic - anything under components/ plus the main/ application component is
#  picked up by IDF, so new components (usb_rtlsdr, demod1090, ...) need no edit
#  here as they land.
# ============================================================================
cmake_minimum_required(VERSION 3.16)

# Pull in the ESP-IDF CMake system. This requires the IDF environment to be
# exported first (the tools/*.bat helpers assume `idf.py` is already on PATH).
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# The project name becomes the firmware base name (adsbin.elf / adsbin.bin).
project(adsbin)
