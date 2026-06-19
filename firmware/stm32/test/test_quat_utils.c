/*
 * test_quat_utils.c — Host-side unit tests for src/servo/j5vr_quat_utils.c
 *
 * Exercises the pure quaternion algebra used by the JONNY5 wrist-orientation
 * control (normalize / Hamilton product / conjugate / Euler & rotation-vector
 * conversions) against known mathematical identities. Runs on a stock host
 * compiler — the Zephyr-only IMU driver is replaced by a tiny type stub.
 */
#include "test_framework.h"
#include "servo/j5vr_quat_utils.h"
#include "spi/j5_protocol.h" /* real, host-friendly: struct j5vr_state */
#include "imu/imu.h"         /* test stub: struct imu_quat              */

/* ---- quat_normalize -------------------------------------------------- */

static void test_normalize_gives_unit_norm(void)
{
    quat_t q = quat_normalize((quat_t){0.5f, 0.5f, 0.5f, 0.5f});
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    T_ASSERT_FLOAT(1.0f, n, 1e-5f);
}

static void test_normalize_zero_returns_identity(void)
{
    quat_t q = quat_normalize((quat_t){0.0f, 0.0f, 0.0f, 0.0f});
    T_ASSERT_FLOAT(1.0f, q.w, 1e-6f);
    T_ASSERT_FLOAT(0.0f, q.x, 1e-6f);
    T_ASSERT_FLOAT(0.0f, q.y, 1e-6f);
    T_ASSERT_FLOAT(0.0f, q.z, 1e-6f);
}

/* ---- quat_multiply (Hamilton product) -------------------------------- */

static void test_multiply_identity_is_neutral(void)
{
    quat_t id = {1.0f, 0.0f, 0.0f, 0.0f};
    quat_t a = {0.7071f, 0.7071f, 0.0f, 0.0f};
    quat_t r = quat_multiply(id, a);
    T_ASSERT_FLOAT(a.w, r.w, 1e-5f);
    T_ASSERT_FLOAT(a.x, r.x, 1e-5f);
    T_ASSERT_FLOAT(a.y, r.y, 1e-5f);
    T_ASSERT_FLOAT(a.z, r.z, 1e-5f);
}

static void test_multiply_i_times_i_is_minus_one(void)
{
    quat_t i = {0.0f, 1.0f, 0.0f, 0.0f};
    quat_t r = quat_multiply(i, i);
    T_ASSERT_FLOAT(-1.0f, r.w, 1e-6f);
    T_ASSERT_FLOAT(0.0f, r.x, 1e-6f);
}

static void test_multiply_i_times_j_is_k(void)
{
    quat_t i = {0.0f, 1.0f, 0.0f, 0.0f};
    quat_t j = {0.0f, 0.0f, 1.0f, 0.0f};
    quat_t r = quat_multiply(i, j);
    T_ASSERT_FLOAT(0.0f, r.w, 1e-6f);
    T_ASSERT_FLOAT(0.0f, r.x, 1e-6f);
    T_ASSERT_FLOAT(0.0f, r.y, 1e-6f);
    T_ASSERT_FLOAT(1.0f, r.z, 1e-6f); /* i ⊗ j = k */
}

/* ---- quat_conjugate -------------------------------------------------- */

static void test_conjugate_flips_vector_part(void)
{
    quat_t r = quat_conjugate((quat_t){1.0f, 2.0f, 3.0f, 4.0f});
    T_ASSERT_FLOAT(1.0f, r.w, 1e-6f);
    T_ASSERT_FLOAT(-2.0f, r.x, 1e-6f);
    T_ASSERT_FLOAT(-3.0f, r.y, 1e-6f);
    T_ASSERT_FLOAT(-4.0f, r.z, 1e-6f);
}

static void test_q_times_conj_is_scalar_norm(void)
{
    /* For a unit quaternion: q ⊗ q* = (1, 0, 0, 0). */
    quat_t q = {0.5f, 0.5f, 0.5f, 0.5f};
    quat_t r = quat_multiply(q, quat_conjugate(q));
    T_ASSERT_FLOAT(1.0f, r.w, 1e-5f);
    T_ASSERT_FLOAT(0.0f, r.x, 1e-5f);
    T_ASSERT_FLOAT(0.0f, r.y, 1e-5f);
    T_ASSERT_FLOAT(0.0f, r.z, 1e-5f);
}

/* ---- quat_to_ypr_deg ------------------------------------------------- */

static void test_ypr_identity_is_zero(void)
{
    float y, p, r;
    quat_to_ypr_deg(1.0f, 0.0f, 0.0f, 0.0f, &y, &p, &r);
    T_ASSERT_FLOAT(0.0f, y, 1e-3f);
    T_ASSERT_FLOAT(0.0f, p, 1e-3f);
    T_ASSERT_FLOAT(0.0f, r, 1e-3f);
}

static void test_ypr_yaw_90_about_z(void)
{
    /* +90° about Z: q = (cos45, 0, 0, sin45). */
    const float c = 0.70710678f;
    float y, p, r;
    quat_to_ypr_deg(c, 0.0f, 0.0f, c, &y, &p, &r);
    T_ASSERT_FLOAT(90.0f, y, 1e-2f);
    T_ASSERT_FLOAT(0.0f, p, 1e-2f);
    T_ASSERT_FLOAT(0.0f, r, 1e-2f);
}

static void test_ypr_null_pointers_are_safe(void)
{
    quat_to_ypr_deg(1.0f, 0.0f, 0.0f, 0.0f, NULL, NULL, NULL);
    T_ASSERT_TRUE(1); /* must not crash on optional outputs */
}

/* ---- rotation-vector / twist: identity → zero ------------------------ */

static void test_rotvec_identity_is_zero(void)
{
    float y, p, r;
    quat_to_rotvec_ypr_deg(1.0f, 0.0f, 0.0f, 0.0f, &y, &p, &r);
    T_ASSERT_FLOAT(0.0f, y, 1e-3f);
    T_ASSERT_FLOAT(0.0f, p, 1e-3f);
    T_ASSERT_FLOAT(0.0f, r, 1e-3f);
}

static void test_twist_identity_is_zero(void)
{
    float y, p, r;
    quat_to_twist_ypr_deg(1.0f, 0.0f, 0.0f, 0.0f, &y, &p, &r);
    T_ASSERT_FLOAT(0.0f, y, 1e-3f);
    T_ASSERT_FLOAT(0.0f, p, 1e-3f);
    T_ASSERT_FLOAT(0.0f, r, 1e-3f);
}

/* ---- constructors from system types ---------------------------------- */

static void test_from_j5vr_copies_components(void)
{
    struct j5vr_state s = {0};
    s.quat_w = 0.1f; s.quat_x = 0.2f; s.quat_y = 0.3f; s.quat_z = 0.4f;
    quat_t q = quat_from_j5vr(&s);
    T_ASSERT_FLOAT(0.1f, q.w, 1e-6f);
    T_ASSERT_FLOAT(0.2f, q.x, 1e-6f);
    T_ASSERT_FLOAT(0.3f, q.y, 1e-6f);
    T_ASSERT_FLOAT(0.4f, q.z, 1e-6f);
}

static void test_from_j5vr_null_is_identity(void)
{
    quat_t q = quat_from_j5vr(NULL);
    T_ASSERT_FLOAT(1.0f, q.w, 1e-6f);
    T_ASSERT_FLOAT(0.0f, q.x, 1e-6f);
}

static void test_from_imu_copies_components(void)
{
    struct imu_quat iq = {0.6f, 0.0f, 0.8f, 0.0f};
    quat_t q = quat_from_imu(&iq);
    T_ASSERT_FLOAT(0.6f, q.w, 1e-6f);
    T_ASSERT_FLOAT(0.8f, q.y, 1e-6f);
}

int main(void)
{
    printf("== j5vr_quat_utils — host unit tests ==\n");
    T_RUN(test_normalize_gives_unit_norm);
    T_RUN(test_normalize_zero_returns_identity);
    T_RUN(test_multiply_identity_is_neutral);
    T_RUN(test_multiply_i_times_i_is_minus_one);
    T_RUN(test_multiply_i_times_j_is_k);
    T_RUN(test_conjugate_flips_vector_part);
    T_RUN(test_q_times_conj_is_scalar_norm);
    T_RUN(test_ypr_identity_is_zero);
    T_RUN(test_ypr_yaw_90_about_z);
    T_RUN(test_ypr_null_pointers_are_safe);
    T_RUN(test_rotvec_identity_is_zero);
    T_RUN(test_twist_identity_is_zero);
    T_RUN(test_from_j5vr_copies_components);
    T_RUN(test_from_j5vr_null_is_identity);
    T_RUN(test_from_imu_copies_components);
    return T_SUMMARY();
}
