# Host-side unit tests for the JONNY5 STM32 firmware.
#
# Compiles the pure firmware logic with a stock host compiler (no board, no
# Zephyr): the Zephyr-only IMU driver header is shadowed by a tiny stub in
# stubs/, while the real protocol/header files under ../src are used as-is.
#
#   make test    build and run
#   make clean   remove the built runner

CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra
# stubs/ MUST come before ../src so imu/imu.h resolves to the host stub
INCLUDES = -Istubs -I../src
LDLIBS   = -lm

QUAT_SRC = ../src/servo/j5vr_quat_utils.c

test_quat: test_quat_utils.c $(QUAT_SRC) test_framework.h stubs/imu/imu.h
	$(CC) $(CFLAGS) $(INCLUDES) test_quat_utils.c $(QUAT_SRC) $(LDLIBS) -o test_quat

.PHONY: test clean
test: test_quat
	./test_quat

clean:
	rm -f test_quat test_quat.exe
