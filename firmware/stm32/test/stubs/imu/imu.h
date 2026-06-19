/*
 * Host-test stub for imu/imu.h.
 *
 * Provides ONLY the type used by j5vr_quat_utils.c (struct imu_quat), so the
 * pure quaternion math can be compiled on the host without pulling in the
 * Zephyr sensor driver headers. On target builds the real imu/imu.h is used.
 */
#ifndef IMU_H_HOST_TEST_STUB
#define IMU_H_HOST_TEST_STUB

struct imu_quat { float w, x, y, z; };

#endif /* IMU_H_HOST_TEST_STUB */
