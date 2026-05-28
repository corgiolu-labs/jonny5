cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/overlays/uart.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/overlays/i2c.overlay
    # mpu6050.overlay dropped: production backend is BNO085, talked to via
    # raw i2c_read/i2c_write from src/imu/bno085.c (no devicetree node needed).
    # The overlay file is kept on disk for reversibility; re-add this line
    # and set CONFIG_MPU6050=y in prj.conf to revert to the MPU6050 backend.
    ${CMAKE_CURRENT_SOURCE_DIR}/overlays/pwm.overlay
)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(JONNY5_3.0)

# Path assoluti: app in src/ con struttura modulare
get_filename_component(APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
target_include_directories(app PRIVATE 
    ${APP_ROOT}/src 
    ${APP_ROOT}/src/spi 
    ${APP_ROOT}/src/uart 
    ${APP_ROOT}/src/core 
    ${APP_ROOT}/src/common
    ${APP_ROOT}/src/servo
    ${APP_ROOT}/src/imu
)

# Compilazione ricorsiva di tutti i file .c in src/
file(GLOB_RECURSE APP_SOURCES ${APP_ROOT}/src/*.c)
target_sources(app PRIVATE ${APP_SOURCES})
