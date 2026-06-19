# Host-side unit tests

Pure-logic unit tests for the STM32 firmware, compiled and run on a **stock host
`gcc`** — no board and no Zephyr RTOS required. The Zephyr-only IMU driver header
is shadowed by a tiny type stub in [`stubs/`](stubs/), so the real algorithms
under [`../src`](../src) are exercised as-is. The harness
([`test_framework.h`](test_framework.h)) is dependency-free.

## Run

```sh
make test        # build + run
make clean
```

or directly:

```sh
gcc -std=c11 -O2 -Wall -Wextra -Istubs -I../src \
    test_quat_utils.c ../src/servo/j5vr_quat_utils.c -lm -o test_quat && ./test_quat
```

## Coverage

| Suite | Module under test | What it checks |
|---|---|---|
| `test_quat_utils.c` | `src/servo/j5vr_quat_utils.c` | quaternion normalize · Hamilton product · conjugate · Euler / rotation-vector / twist conversions, against known identities (i⊗i = −1, i⊗j = k, q⊗q\* = 1, yaw 90° about Z, null-pointer safety) |

These run automatically in CI on every push that touches `firmware/stm32/`
(see [`.github/workflows/ci.yml`](../../../.github/workflows/ci.yml)).
