/*
 * JONNY5-4.0 - IMU Module (MPU6050)
 * Lettura dati accelerometro e giroscopio, calcolo quaternione via DMP.
 *
 * NOTE [Refactor-Phase1]:
 *   - Le funzioni sul critical path (imu_update_orientation, imu_get_snapshot,
 *     imu_get_quaternion_try, ecc.) NON devono essere modificate nei refactor.
 *   - Le funzioni marcate nei report come DIAGNOSTIC_ONLY (es. probe I2C,
 *     logging esteso) possono essere solo documentate o raggruppate in blocchi
 *     commentati "diagnostica", senza rimuoverle né cambiare il comportamento.
 */

#include "imu.h"
#include "imu/bno085.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_DBG);

/* Serve per poter richiamare PINCTRL_DT_DEV_CONFIG_GET(i2c1) in questo TU */
#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
PINCTRL_DT_DEFINE(DT_NODELABEL(i2c1));
#endif

/* Stato quaternione IMU (Madgwick) — inizialmente identità */
static struct imu_quat g_imu_quat = {
	.w = 1.0f,
	.x = 0.0f,
	.y = 0.0f,
	.z = 0.0f,
};

/* Parametro Madgwick (beta) — adattivo in base alla norma del giroscopio:
 *   fermo  (|gyro| < BETA_GYRO_LOW):  beta = BETA_STILL  → meno rumore
 *   mosso  (|gyro| > BETA_GYRO_HIGH): beta = BETA_MOVING → risposta rapida
 *   zona di transizione lineare tra i due estremi.
 *
 *   Valori empirici (MPU6050 @ ±250 dps, bias-compensato):
 *     BETA_STILL  = 0.02  → std roll/pitch ~0.02°  (era 0.10° con beta fisso 0.25)
 *     BETA_MOVING = 0.25  → risposta identica a prima durante il movimento
 *     soglia fermo:  5 °/s  = 0.087 rad/s
 *     soglia mosso: 20 °/s  = 0.349 rad/s
 */
#define BETA_STILL       0.02f
#define BETA_MOVING      0.25f
#define BETA_GYRO_LOW    0.087f   /* rad/s — ~5 deg/s  */
#define BETA_GYRO_HIGH   0.349f   /* rad/s — ~20 deg/s */

static float g_madgwick_beta = BETA_MOVING;

/* True quando l'ultimo update Madgwick è basato su campioni validi */
static bool g_imu_orientation_valid = false;

/* Ultimo gyro usato in Madgwick */
static float g_last_gyro_x = 0.0f;
static float g_last_gyro_y = 0.0f;
static float g_last_gyro_z = 0.0f;

/* Bias del giroscopio (rad/s) stimato alla calibrazione di avvio.
 * Sottratto dai campioni raw prima di Madgwick per ridurre il drift yaw. */
static float g_gyro_bias_x = 0.0f;
static float g_gyro_bias_y = 0.0f;
static float g_gyro_bias_z = 0.0f;
static bool  g_gyro_bias_calibrated = false;

/* Numero campioni per la calibrazione bias (a 400 Hz = ~1.25 secondi) */
#define GYRO_CALIB_SAMPLES 500

/* Forward declaration: imu_read è definita più in basso nel file */
static int imu_read(struct imu_data *data);

/*
 * Double-buffer atomico lock-free per snapshot IMU.
 *
 * Schema writer/reader senza mutex, senza seqlock, senza retry:
 *   Writer (thread IMU, 400 Hz):
 *     1. Legge l'indice attivo corrente (0 o 1).
 *     2. Scrive il nuovo snapshot nel buffer inattivo (indice ^ 1).
 *     3. Esegue atomic_set() per pubblicare il nuovo indice attivo.
 *        Da questo punto tutti i nuovi reader vedono il buffer aggiornato.
 *   Reader (thread SPI, 60–100 Hz):
 *     1. atomic_get() dell'indice attivo — una sola lettura atomica.
 *     2. Copia strutturale del buffer indicato — nessun loop, nessuna
 *        dipendenza dal writer.
 *
 * Sostituisce il vecchio seqlock (g_imu_seq pari/dispari + __DMB()),
 * introdotto per eliminare imu_valid=False sporadici a SPI > 60 Hz
 * causati dalla contesa writer/reader sul seqlock dispari.
 * Con il double-buffer imu_valid=100% è garantito fino a 100 Hz SPI.
 */
static imu_snapshot_t g_imu_buffers[2];
static atomic_t       g_imu_active_idx       = ATOMIC_INIT(0);
static bool           g_imu_has_valid_snapshot = false;
static uint32_t       g_imu_sample_counter = 0;

/* Device tree node per MPU6050 */
#define MPU6050_NODE DT_ALIAS(mpu6050)
#if !DT_NODE_EXISTS(MPU6050_NODE)
#warning "MPU6050 node not found in device tree. IMU will be disabled."
#define MPU6050_NODE DT_INVALID_NODE
static const struct device *mpu6050_dev = NULL;
#else
static const struct device *mpu6050_dev = DEVICE_DT_GET(MPU6050_NODE);
#endif
static bool imu_available = false;

/* I2C1 device (lettura raw registri MPU6050, single-owner in imu_read) */
#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
static const struct device *g_i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
#else
static const struct device *g_i2c1_dev = NULL;
#endif

/* MPU6050 register map (subset) */
#define MPU6050_ADDR      0x68
#define MPU6050_REG_ACCEL 0x3B  /* accel_x..gyro_z + temp */

static inline int16_t be16s(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] << 8 | (uint16_t)p[1]);
}


void imu_i2c_bus_recovery(void)
{
	/* AN4559/ST: I2C bus recovery tramite 9 clock manuali su SCL */
	LOG_WRN("[IMU] I2C bus recovery start");

	/* Nucleo-F446RE: I2C1 su PB8(SCL) / PB9(SDA) come da overlay */
#if DT_NODE_EXISTS(DT_NODELABEL(gpiob)) && DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	const struct device *gpio_b = DEVICE_DT_GET(DT_NODELABEL(gpiob));
	if (!device_is_ready(gpio_b))
	{
		LOG_ERR("[IMU] GPIOB not ready (cannot recover)");
		return;
	}

	/* Configura temporaneamente PB8/PB9 come GPIO open-drain */
	(void)gpio_pin_configure(gpio_b, 8, GPIO_OUTPUT | GPIO_OPEN_DRAIN);
	(void)gpio_pin_configure(gpio_b, 9, GPIO_OUTPUT | GPIO_OPEN_DRAIN);
	/* Rilascia SDA (open-drain high = high-Z) */
	gpio_pin_set(gpio_b, 9, 1);

	/* Clock pulses — k_usleep invece di k_busy_wait: cede la CPU agli altri thread */
	for (int i = 0; i < 9; i++)
	{
		gpio_pin_set(gpio_b, 8, 1);
		k_usleep(10);
		gpio_pin_set(gpio_b, 8, 0);
		k_usleep(10);
	}

	/* Termina con SCL alto */
	gpio_pin_set(gpio_b, 8, 1);
	k_usleep(10);

	/* Verifica SDA dopo i 9 clock */
	int sda = gpio_pin_get(gpio_b, 9);
	if (sda == 0)
	{
		LOG_ERR("[IMU] I2C bus still locked after recovery");
	}

	/* Sequenza START+STOP manuale per resettare lo state-machine del
	 * device I2C (NXP/ST AN4509). I soli 9 clock non bastano se lo slave
	 * sta ancora "aspettando il prossimo byte": serve una transizione
	 * START (SDA 1->0 con SCL=1) seguita da STOP (SDA 0->1 con SCL=1) per
	 * riportare entrambi i lati in idle. */
	gpio_pin_set(gpio_b, 9, 1);
	gpio_pin_set(gpio_b, 8, 1);
	k_usleep(10);
	gpio_pin_set(gpio_b, 9, 0);      /* START: SDA 1->0 mentre SCL=1 */
	k_usleep(10);
	gpio_pin_set(gpio_b, 8, 0);      /* abbassa SCL dopo START */
	k_usleep(10);
	gpio_pin_set(gpio_b, 8, 1);      /* risale SCL con SDA basso */
	k_usleep(10);
	gpio_pin_set(gpio_b, 9, 1);      /* STOP: SDA 0->1 mentre SCL=1 */
	k_usleep(10);

	/* Ripristina pinctrl I2C1 default */
	(void)pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(i2c1)), PINCTRL_STATE_DEFAULT);
#else
	LOG_WRN("[IMU] I2C bus recovery skipped (missing DT nodes)");
#endif

	LOG_WRN("[IMU] I2C bus recovery done");
}

/* Set to true by the bus scan when a BNO085 (0x4A or 0x4B) is found on I2C1.
 * Consumed by imu_init() to stop the MPU6050 retry loop so the BNO085 driver
 * has exclusive access to the bus. */
static bool s_bno085_detected = false;

/* Diagnostic-only: scan I2C bus and log every address that ACKs.
 * Useful when the sensor on the bus is unknown or has been swapped (e.g. BNO085
 * at 0x4A/0x4B in place of MPU6050 at 0x68). Uses a 1-byte raw i2c_read which
 * is protocol-agnostic: any device present must ACK its address.
 */
static void imu_i2c_scan_bus(const struct device *bus, const char *name)
{
	if (bus == NULL || !device_is_ready(bus)) {
		LOG_WRN("[IMU] %s SCAN skipped: bus not ready", name);
		return;
	}
	char line[160];
	int n = snprintf(line, sizeof(line), "[IMU] %s SCAN 0x08-0x77:", name);
	int hits = 0;
	for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t b = 0;
		int r = i2c_read(bus, &b, 1, addr);
		if (r == 0) {
			if (n < (int)sizeof(line) - 8) {
				n += snprintf(line + n, sizeof(line) - n,
					      " 0x%02X", (unsigned)addr);
			}
			hits++;
			if (addr == 0x4A || addr == 0x4B) {
				s_bno085_detected = true;
			}
		}
	}
	if (hits == 0) {
		LOG_WRN("[IMU] %s SCAN: no devices ACKed", name);
	} else {
		LOG_INF("%s  (%d device(s))", line, hits);
	}
}

static void imu_i2c_probe_log(void)
{
	/* WHO_AM_I register = 0x75; atteso 0x68 per MPU6050 */
	const uint8_t reg_whoami = 0x75;

#define IMU_PROBE_BUS(_name, _nodelabel)                                      \
	do {                                                                  \
		if (!DT_NODE_EXISTS(DT_NODELABEL(_nodelabel))) {              \
			break;                                                \
		}                                                             \
		const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(_nodelabel)); \
		if (!device_is_ready(i2c_dev)) {                               \
			LOG_ERR("[IMU] %s not ready (cannot probe)", _name);   \
			break;                                                \
		}                                                             \
		uint8_t who = 0;                                               \
		int r = i2c_reg_read_byte(i2c_dev, 0x68, reg_whoami, &who);     \
		if (r == 0) {                                                  \
			LOG_INF("[IMU] %s PROBE addr=0x68 WHO_AM_I=0x%02x", _name, who); \
		} else {                                                      \
			LOG_WRN("[IMU] %s PROBE addr=0x68 failed (%d)", _name, r); \
		}                                                             \
		who = 0;                                                      \
		r = i2c_reg_read_byte(i2c_dev, 0x69, reg_whoami, &who);        \
		if (r == 0) {                                                 \
			LOG_INF("[IMU] %s PROBE addr=0x69 WHO_AM_I=0x%02x", _name, who); \
		} else {                                                      \
			LOG_WRN("[IMU] %s PROBE addr=0x69 failed (%d)", _name, r); \
		}                                                             \
	} while (0)

	/* Questa probe è chiamata da imu_init() → thread IMU (prio 6, deferred).
	 * Su I2C1 è sempre attiva; I2C2/I2C3 sono disabilitati di default
	 * per evitare timeout su bus non usati che potrebbero ritardare l'init. */
#ifndef IMU_PROBE_ALL_BUSES
#define IMU_PROBE_ALL_BUSES 0
#endif

	IMU_PROBE_BUS("I2C1", i2c1);
#if IMU_PROBE_ALL_BUSES
	IMU_PROBE_BUS("I2C2", i2c2);
	IMU_PROBE_BUS("I2C3", i2c3);
#endif

#undef IMU_PROBE_BUS
}

static void madgwick_update(float dt,
			    float gx, float gy, float gz,
			    float ax, float ay, float az)
{
	/* Implementazione Madgwick 6DOF (gyro+accel), formulazione standard.
	 * Assunzioni:
	 * - gyro in rad/s
	 * - accel normalizzato (vettore unitario)
	 */
	float q0 = g_imu_quat.w;
	float q1 = g_imu_quat.x;
	float q2 = g_imu_quat.y;
	float q3 = g_imu_quat.z;

	/* Se accel non valido, integra solo gyro */
	const float acc_norm_sq = ax * ax + ay * ay + az * az;
	const bool acc_valid = (acc_norm_sq > 1e-6f);

	/* Rate of change of quaternion from gyroscope */
	float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	if (acc_valid)
	{
		/* Gradient descent corrective step */
		const float _2q0 = 2.0f * q0;
		const float _2q1 = 2.0f * q1;
		const float _2q2 = 2.0f * q2;
		const float _2q3 = 2.0f * q3;

		const float _4q0 = 4.0f * q0;
		const float _4q1 = 4.0f * q1;
		const float _4q2 = 4.0f * q2;

		const float _8q1 = 8.0f * q1;
		const float _8q2 = 8.0f * q2;

		const float q0q0 = q0 * q0;
		const float q1q1 = q1 * q1;
		const float q2q2 = q2 * q2;
		const float q3q3 = q3 * q3;

		/* s = gradient of objective function */
		float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

		/* Normalizza step */
		const float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		if (sn > 1e-9f)
		{
			s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn;
			/* Apply feedback step */
			qDot0 -= g_madgwick_beta * s0;
			qDot1 -= g_madgwick_beta * s1;
			qDot2 -= g_madgwick_beta * s2;
			qDot3 -= g_madgwick_beta * s3;
		}
	}

	/* Integrate to yield quaternion */
	q0 += qDot0 * dt;
	q1 += qDot1 * dt;
	q2 += qDot2 * dt;
	q3 += qDot3 * dt;

	/* Normalize quaternion */
	const float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (qn > 1e-9f)
	{
		q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn;
	}
	else
	{
		q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	}

	g_imu_quat.w = q0;
	g_imu_quat.x = q1;
	g_imu_quat.y = q2;
	g_imu_quat.z = q3;
}

/**
 * Calibrazione bias giroscopio: campiona GYRO_CALIB_SAMPLES letture a sensore fermo
 * e calcola la media per ogni asse. Il bias viene sottratto in imu_update_orientation()
 * prima di passare i dati a Madgwick. Riduce il drift yaw da ~1.7 deg/s a < 0.05 deg/s.
 *
 * Precondizione: I2C e sensore già verificati (chiamare dopo WHO_AM_I ok).
 * Durata: ~1.25 secondi a 400 Hz (GYRO_CALIB_SAMPLES=500 × 2.5ms).
 */
static void imu_calibrate_gyro_bias(void)
{
	struct imu_data d;
	double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
	int valid = 0;

	LOG_INF("[IMU] Calibrazione bias gyro: %d campioni (~%.1f s)...",
		GYRO_CALIB_SAMPLES, (double)(GYRO_CALIB_SAMPLES * 0.0025f));

	for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
		if (imu_read(&d) == 0) {
			sum_x += (double)d.gyro_x;
			sum_y += (double)d.gyro_y;
			sum_z += (double)d.gyro_z;
			valid++;
		}
		k_usleep(2500); /* 2.5 ms = 400 Hz */
	}

	if (valid < GYRO_CALIB_SAMPLES / 2) {
		LOG_WRN("[IMU] Calibrazione bias fallita: solo %d/%d campioni validi",
			valid, GYRO_CALIB_SAMPLES);
		g_gyro_bias_x = 0.0f;
		g_gyro_bias_y = 0.0f;
		g_gyro_bias_z = 0.0f;
		g_gyro_bias_calibrated = false;
		return;
	}

	g_gyro_bias_x = (float)(sum_x / valid);
	g_gyro_bias_y = (float)(sum_y / valid);
	g_gyro_bias_z = (float)(sum_z / valid);
	g_gyro_bias_calibrated = true;

	const float rad2deg = 180.0f / 3.14159265f;
	LOG_INF("[IMU] Bias gyro calibrato (%d campioni): "
		"gx=%+.4f gy=%+.4f gz=%+.4f rad/s  "
		"(gz=%+.3f deg/s)",
		valid,
		(double)g_gyro_bias_x, (double)g_gyro_bias_y, (double)g_gyro_bias_z,
		(double)(g_gyro_bias_z * rad2deg));
}

int imu_init(void)
{
	/* Reset internal quat + sample counter regardless of which backend we
	 * end up using. Double-buffer remains stale until first publish. */
	g_imu_orientation_valid = false;
	g_imu_sample_counter = 0;
	g_imu_quat.w = 1.0f;
	g_imu_quat.x = 0.0f;
	g_imu_quat.y = 0.0f;
	g_imu_quat.z = 0.0f;

	/* Bus recovery: good hygiene for any I2C device after reset/flash. */
	imu_i2c_bus_recovery();

	/* Diagnostic bus scan. Sets s_bno085_detected when 0x4A/0x4B ACKs.
	 * Logs every address that responds — valuable when the sensor on the
	 * bus changes (MPU6050 at 0x68, BNO085 at 0x4A/0x4B, etc.). */
#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	imu_i2c_scan_bus(DEVICE_DT_GET(DT_NODELABEL(i2c1)), "I2C1");
#endif

	/* ============================================================ *
	 * Production backend: BNO085 (fused rotation vector via SHTP).
	 * ============================================================ */
	if (s_bno085_detected) {
		int rc = bno085_init();
		if (rc != 0) {
			imu_available = false;
			LOG_ERR("[IMU] bno085_init failed rc=%d (imu_available=0)", rc);
			return rc;
		}
		imu_available = true;
		LOG_INF("[IMU] init ok (backend=BNO085 @ 0x%02X) — quat feed from Rotation Vector",
			BNO085_I2C_ADDR);
		return 0;
	}

	/* ============================================================ *
	 * Legacy backend: MPU6050 raw + Madgwick (retained for
	 * reversibility; inactive while a BNO085 is present on I2C1).
	 * ============================================================ */
#if DT_NODE_EXISTS(MPU6050_NODE)
	int whoami_rc = -1;
	uint8_t whoami_val = 0;

#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	{
		const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
		const uint8_t reg_whoami = 0x75;
		uint8_t who = 0;

		if (!device_is_ready(i2c1_dev)) {
			imu_available = false;
			LOG_ERR("[IMU] init failed: I2C1 not ready (imu_available=0)");
			return -ENODEV;
		}

		int r = i2c_reg_read_byte(i2c1_dev, 0x68, reg_whoami, &who);
		whoami_rc = r;
		whoami_val = who;
		if (r != 0 || who != 0x68) {
			imu_available = false;
			LOG_ERR("[IMU] init failed: WHO_AM_I read failed (r=%d who=0x%02x) (imu_available=0)", r, who);
			return -EIO;
		}
		imu_available = true;
	}
#endif

	imu_i2c_probe_log();

	LOG_INF("[IMU_INIT] mpu6050_dev ready=%d imu_available=%d whoami=0x%02X i2c_reg_read_rc=%d",
		device_is_ready(mpu6050_dev) ? 1 : 0, imu_available ? 1 : 0, whoami_val, whoami_rc);

	if (!device_is_ready(mpu6050_dev)) {
		LOG_WRN("[IMU] MPU6050 device not ready (driver init failed at boot?) — continuing due to WHO_AM_I ok");
	}

	if (imu_available) {
		LOG_INF("[IMU] init ok (backend=MPU6050): device_is_ready=%d",
			device_is_ready(mpu6050_dev) ? 1 : 0);
	}

	if (imu_available && mpu6050_dev != NULL && device_is_ready(mpu6050_dev)) {
		int acc_rc = sensor_sample_fetch_chan(mpu6050_dev, SENSOR_CHAN_ACCEL_XYZ);
		int gyro_rc = sensor_sample_fetch_chan(mpu6050_dev, SENSOR_CHAN_GYRO_XYZ);
		LOG_INF("[IMU_INIT] sensor_sample_fetch accel_rc=%d gyro_rc=%d", acc_rc, gyro_rc);
	}

	imu_calibrate_gyro_bias();
	return 0;
#else
	LOG_ERR("[IMU] no supported IMU detected on I2C1 (no BNO085 and no MPU6050 DT node) — imu_available=0");
	imu_available = false;
	return -ENODEV;
#endif
}

static int imu_read(struct imu_data *data)
{
	if (!imu_available || !data) {
		return -EINVAL;
	}

	/* Single-owner + no-deadlock:
	 * Evita completamente sensor_sample_fetch_chan/driver MPU6050, che può bloccare.
	 * Legge i registri raw via I2C burst read (14 byte).
	 */
	if (g_i2c1_dev == NULL || !device_is_ready(g_i2c1_dev)) {
		return -ENODEV;
	}

	/* Fail-counter: se il bus si blocca (SDA stuck, MPU6050 in stato incoerente),
	 * dopo N errori consecutivi tentiamo un bus recovery invece di lasciare il
	 * pill LED a flickerare on/off indefinitamente. Reset su read ok. */
	static uint32_t s_fail_count = 0;
	uint8_t raw[14];
	int rc = i2c_burst_read(g_i2c1_dev, MPU6050_ADDR, MPU6050_REG_ACCEL, raw, sizeof(raw));
	if (rc != 0) {
		s_fail_count++;
		if (s_fail_count >= 50U) { /* ~125 ms @ 400 Hz */
			s_fail_count = 0;
			imu_i2c_bus_recovery();
		}
		return -EIO;
	}
	s_fail_count = 0;

	const int16_t ax = be16s(&raw[0]);
	const int16_t ay = be16s(&raw[2]);
	const int16_t az = be16s(&raw[4]);
	const int16_t t  = be16s(&raw[6]);
	const int16_t gx = be16s(&raw[8]);
	const int16_t gy = be16s(&raw[10]);
	const int16_t gz = be16s(&raw[12]);

	/* Conversioni (assumendo range default dopo reset: accel ±2g, gyro ±250 dps) */
	const float g0 = 9.80665f;
	const float accel_lsb_per_g = 16384.0f;
	const float gyro_lsb_per_dps = 131.0f;
	const float deg2rad = 3.14159265358979323846f / 180.0f;

	data->accel_x = ((float)ax / accel_lsb_per_g) * g0;
	data->accel_y = ((float)ay / accel_lsb_per_g) * g0;
	data->accel_z = ((float)az / accel_lsb_per_g) * g0;

	data->gyro_x = ((float)gx / gyro_lsb_per_dps) * deg2rad;
	data->gyro_y = ((float)gy / gyro_lsb_per_dps) * deg2rad;
	data->gyro_z = ((float)gz / gyro_lsb_per_dps) * deg2rad;

	/* Temp: MPU6050 datasheet: Temp_in_C = (raw/340) + 36.53 */
	data->temp = ((float)t / 340.0f) + 36.53f;

	return 0;
}

bool imu_is_available(void)
{
	return imu_available;
}

bool imu_is_orientation_valid(void)
{
	return g_imu_orientation_valid;
}

/**
 * Azzera lo stato di orientamento (quaternione + snapshot) quando IMU è disabilitata (IMUOFF).
 * Evita che la telemetria o altri consumer riutilizzino l'ultimo quat valido.
 */
void imu_clear_orientation_state(void)
{
	g_imu_orientation_valid = false;
	g_imu_quat.w = 1.0f;
	g_imu_quat.x = 0.0f;
	g_imu_quat.y = 0.0f;
	g_imu_quat.z = 0.0f;

	/* Scrivi snapshot di reset su entrambi i buffer per coerenza */
	imu_snapshot_t reset = {
		.quat_w = 1.0f, .quat_x = 0.0f, .quat_y = 0.0f, .quat_z = 0.0f,
		.zworld_x = 0.0f, .zworld_y = 0.0f, .zworld_z = 1.0f,
		.wrist_roll_rate = 0.0f, .wrist_pitch_rate = 0.0f, .wrist_yaw_rate = 0.0f,
	};
	g_imu_buffers[0] = reset;
	g_imu_buffers[1] = reset;
	atomic_set(&g_imu_active_idx, 0);
	g_imu_has_valid_snapshot = false;
	g_imu_sample_counter = 0;
}

/* ------------------------------------------------------------------------- *
 * imu_update_orientation — production body (BNO085 backend).
 *
 * Called at 400 Hz by the IMU thread. BNO085 emits Rotation Vector reports
 * natively at ~100 Hz, so roughly 3 of every 4 calls return no new data —
 * those early-exit without re-publishing, and the snapshot stays valid from
 * the previous publish (downstream continues reading the last fresh quat).
 *
 * imu_snapshot_t mapping in BNO085 v1:
 *   quat_*        = Rotation Vector (unit quaternion, from SH-2 sensor 0x05)
 *   zworld_*      = derived from quat (same formula as legacy)
 *   sample_counter/timestamp_us = advanced on each new packet
 *   accel_*, gyro_*, temp, wrist_*_rate = zeroed (see BNO085-v1 note below)
 *
 * BNO085-v1 temporary limitation: only Rotation Vector is enabled on the
 * sensor; raw accel/gyro/temp and wrist rates are not yet plumbed. Consumers
 * that only read quat_* + zworld_* + sample_counter + timestamp_us are
 * unaffected (this is the SPI telemetry, ws_handlers_imu, dashboard IMU view,
 * VR head-tracking path — verified from current repo grep). If a consumer
 * starts using accel/gyro, Phase 6 will enable Calibrated Accel (sensor 0x01)
 * + Calibrated Gyro (sensor 0x02) on the BNO085 and populate those fields.
 * ------------------------------------------------------------------------- */
void imu_update_orientation(float dt_s)
{
	ARG_UNUSED(dt_s); /* BNO085 is fused internally; Madgwick's dt no longer used */

	if (!imu_available) {
		g_imu_orientation_valid = false;
		return;
	}

	float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
	uint8_t accuracy = 0;
	const int r = bno085_poll_quat(&qw, &qx, &qy, &qz, &accuracy);
	if (r <= 0) {
		/* No new Rotation Vector report this tick (r==0 = empty queue;
		 * r<0 = transient I/O error). Keep the last published snapshot
		 * valid: returning without touching g_imu_orientation_valid
		 * preserves it if it was already true, and returning without
		 * re-publishing keeps the existing active buffer in place. */
		return;
	}

	/* Sync the legacy g_imu_quat (imu_get_quaternion reads it for consumers
	 * that don't use imu_get_snapshot). */
	g_imu_quat.w = qw;
	g_imu_quat.x = qx;
	g_imu_quat.y = qy;
	g_imu_quat.z = qz;
	g_imu_orientation_valid = true;

	/* Double-buffer writer: fill the inactive buffer, then atomic flip. */
	uint32_t cur = (uint32_t)atomic_get(&g_imu_active_idx);
	uint32_t next = cur ^ 1U;

	imu_snapshot_t *dst = &g_imu_buffers[next];
	const uint32_t sample_counter = ++g_imu_sample_counter;
	const uint32_t timestamp_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());

	/* BNO085-v1: raw IMU fields not yet enabled on the sensor. */
	dst->accel_x = 0.0f;
	dst->accel_y = 0.0f;
	dst->accel_z = 0.0f;
	dst->gyro_x  = 0.0f;
	dst->gyro_y  = 0.0f;
	dst->gyro_z  = 0.0f;
	dst->temp    = 0.0f;

	dst->quat_w  = qw;
	dst->quat_x  = qx;
	dst->quat_y  = qy;
	dst->quat_z  = qz;
	dst->zworld_x = 2.0f * (qw * qy + qx * qz);
	dst->zworld_y = 2.0f * (qy * qz - qw * qx);
	dst->zworld_z = qw * qw - qx * qx - qy * qy + qz * qz;

	/* BNO085-v1: wrist rates derived from raw gyro — zeroed until Phase 6
	 * enables Calibrated Gyro on the sensor. */
	dst->wrist_roll_rate  = 0.0f;
	dst->wrist_pitch_rate = 0.0f;
	dst->wrist_yaw_rate   = 0.0f;

	dst->sample_counter = sample_counter;
	dst->timestamp_us = timestamp_us;

	atomic_set(&g_imu_active_idx, (atomic_val_t)next);
	g_imu_has_valid_snapshot = true;

	/* Observability: log BNO085 Rotation-Vector accuracy (0..3) on change so
	 * magnetometer-calibration state is visible on COM3. No SPI/WS effect. */
	{
		static uint8_t s_last_bno085_acc = 0xFF;
		if (accuracy != s_last_bno085_acc) {
			LOG_INF("[IMU] BNO085 accuracy=%u", (unsigned)accuracy);
			s_last_bno085_acc = accuracy;
		}
	}
}

void imu_get_quaternion(struct imu_quat *out)
{
	if (out == NULL)
	{
		return;
	}

	/* Double-buffer reader: atomic_get garantisce visibilità, nessun retry */
	uint32_t idx = (uint32_t)atomic_get(&g_imu_active_idx);
	out->w = g_imu_buffers[idx].quat_w;
	out->x = g_imu_buffers[idx].quat_x;
	out->y = g_imu_buffers[idx].quat_y;
	out->z = g_imu_buffers[idx].quat_z;
}

bool imu_get_snapshot(imu_snapshot_t *out)
{
	if (out == NULL || !g_imu_has_valid_snapshot)
	{
		return false;
	}

	uint32_t idx = (uint32_t)atomic_get(&g_imu_active_idx);
	*out = g_imu_buffers[idx];
	return true;
}

