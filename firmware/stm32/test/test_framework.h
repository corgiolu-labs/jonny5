/*
 * test_framework.h — Minimal dependency-free host-side unit-test harness.
 *
 * No external libraries: a few assert macros + a pass/fail counter, so the
 * pure firmware logic can be compiled and exercised with a stock host gcc
 * (and in CI) without a target board or the Zephyr RTOS.
 */
#ifndef J5_TEST_FRAMEWORK_H
#define J5_TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

static int j5_checks_run = 0;
static int j5_checks_failed = 0;

#define T_CHECK(cond, msg)                                                 \
    do {                                                                   \
        j5_checks_run++;                                                   \
        if (!(cond)) {                                                     \
            j5_checks_failed++;                                            \
            printf("  [FAIL] %s:%d: %s\n", __FILE__, __LINE__, (msg));     \
        }                                                                  \
    } while (0)

#define T_ASSERT_TRUE(cond) T_CHECK((cond), #cond)

#define T_ASSERT_FLOAT(expected, actual, tol)                              \
    T_CHECK(fabsf((float)(expected) - (float)(actual)) <= (float)(tol),    \
            #actual " ~= " #expected)

#define T_RUN(testfn)                                                      \
    do {                                                                   \
        printf("RUN   %s\n", #testfn);                                     \
        testfn();                                                          \
    } while (0)

#define T_SUMMARY()                                                        \
    (printf("\n%d checks, %d failed\n", j5_checks_run, j5_checks_failed),  \
     (j5_checks_failed == 0 ? 0 : 1))

#endif /* J5_TEST_FRAMEWORK_H */
