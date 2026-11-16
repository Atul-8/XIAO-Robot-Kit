/* SPDX-License-Identifier: MIT */
/**
 * @file  car_imu.cpp
 * @brief Mahony AHRS + Euler angles, ported from 06_IMU_Euler.
 *
 * The fusion, calibration and still-tracker code below is the reference code
 * with only these mechanical changes:
 *   - the reference assumed a LEVEL board (calibration removed 1 g from the
 *     dominant axis, so the fused "up" was always the sensor's own axis). The
 *     module instead uses the measured gravity direction as-is and seeds the
 *     quaternion from it, so the board may be mounted at any angle: yaw stays
 *     the rotation about the true vertical, which is what a car needs
 *   - state lives in the class instead of file-scope globals
 *   - the integration step is taken from the hardware heartbeat (exact), with a
 *     micros() fallback when no heartbeat is attached
 *   - yaw is also kept unwrapped for the heading loop
 *   - calibration returns whether the board was still (extra information only)
 */

#include "src/hw/car_imu.h"
#include <Wire.h>
#include <math.h>

/* ── sensor ── */
#define ADDR       0x6A
#define WHO_AM_I   0x0F
#define CTRL1_XL   0x10
#define CTRL2_G    0x11
#define CTRL3_C    0x12
#define OUTX_L_G   0x22
#define OUTX_L_XL  0x28

#define PIN_SDA    5      /* D4 */
#define PIN_SCL    6      /* D5 */

#define ACCEL_LSB  0.000061f  /* +-2g, g/LSB */
#define GYRO_LSB   0.070f     /* +-2000dps, dps/LSB */

#define ODR_HZ     104
#define PERIOD_US  (1000000u / ODR_HZ)

#define KP         2.0f
#define KI         0.0f
#define STILL_DPS  1.0f    /* sum|gyro| under this = still */
#define STILL_MS   1500    /* still time before auto-null */
#define BIAS_ALPHA 0.01f   /* bias leak per tick */
#define CALIB_STILL_DPS 3.0f  /* per-sample spread accepted while calibrating */
#define DT_ACC     0.008f  /* fusion period: 125 Hz, i.e. just above the 104 Hz ODR.
                              The heartbeat runs at 500 Hz, so 4-5 ticks accumulate
                              into one integration step - that is intentional: the
                              sensor cannot deliver data faster than its ODR, and
                              accelerating the filter beyond it only adds noise. */
#define DT_MAX     0.050f  /* longer stall = skip the step (not a rotation) */

#define RAD2DEG    57.29578f
#define PI_F       3.14159265f   /* core already defines PI */

/* ── I2C ── */
static void wr(uint8_t r, uint8_t v) {
    Wire.beginTransmission(ADDR);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission(true);
}

static bool rd(uint8_t r, uint8_t *b, uint8_t n) {
    Wire.beginTransmission(ADDR);
    Wire.write(r);
    if (Wire.endTransmission(false)) return false;
    if (Wire.requestFrom((int)ADDR, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; i++) b[i] = (uint8_t)Wire.read();
    return true;
}

static int16_t i16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* ── quaternion helpers (verbatim) ── */
void CarImu::quatIdentity() { _m.q.w = 1.0f; _m.q.x = _m.q.y = _m.q.z = 0.0f; }

void CarImu::quatNormalize() {
    float n = sqrtf(_m.q.w * _m.q.w + _m.q.x * _m.q.x + _m.q.y * _m.q.y + _m.q.z * _m.q.z);
    if (n < 1e-6f) { quatIdentity(); return; }
    _m.q.w /= n; _m.q.x /= n; _m.q.y /= n; _m.q.z /= n;
}

/* quaternion -> ZYX euler (rad) */
void CarImu::quatToEuler(float *roll, float *pitch, float *yaw) {
    float w = _m.q.w, x = _m.q.x, qy = _m.q.y, z = _m.q.z;
    *roll  = atan2f(2.0f * (w * x + qy * z), 1.0f - 2.0f * (x * x + qy * qy));
    float sp = 2.0f * (w * qy - z * x);
    *pitch = (sp >= 1.0f) ? PI_F * 0.5f : (sp <= -1.0f) ? -PI_F * 0.5f : asinf(sp);
    *yaw   = atan2f(2.0f * (w * z + x * qy), 1.0f - 2.0f * (qy * qy + z * z));
}

void CarImu::mahonyInit() {
    quatIdentity();
    _m.kp = KP;
    _m.ki = KI;
    _m.dt = 1.0f / ODR_HZ;
    _m.integral[0] = _m.integral[1] = _m.integral[2] = 0.0f;
}

/* Reset the filter at the attitude that the measured gravity vector describes:
 * the shortest-arc rotation which maps `up` (body frame) onto world +Z. With a
 * level board this is the identity, so the reference behaviour is unchanged;
 * with a tilted or upside-down mount the filter starts at the true attitude
 * instead of converging into it (that transient used to be pure yaw error,
 * because yaw is not observable from gravity). */
void CarImu::mahonyInitFromUp(float ux, float uy, float uz) {
    mahonyInit();

    float c = ux * ux + uy * uy + uz * uz;
    if (c < 1e-6f) return;                       /* no gravity seen: keep identity */
    c = sqrtf(c);
    ux /= c; uy /= c; uz /= c;

    float dot = uz;                              /* up . (0,0,1) */
    if (dot > 1.0f)  dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    const float ang = acosf(dot);                /* tilt of the mount */
    if (ang < 1e-4f) return;                     /* level: identity is exact */

    /* axis = up x (0,0,1) = (uy, -ux, 0), already horizontal */
    float ax = uy, ay = -ux;
    float n = sqrtf(ax * ax + ay * ay);
    if (n < 1e-6f) {                             /* upside down: 180 deg about X */
        _m.q.w = 0.0f; _m.q.x = 1.0f; _m.q.y = 0.0f; _m.q.z = 0.0f;
        return;
    }
    ax /= n; ay /= n;
    const float s = sinf(ang * 0.5f), cc = cosf(ang * 0.5f);
    _m.q.w = cc; _m.q.x = ax * s; _m.q.y = ay * s; _m.q.z = 0.0f;
    quatNormalize();
}

/* Mahony complementary filter (verbatim: correction is rad/s, gyro is dps) */
void CarImu::mahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz) {
    const float dt = _m.dt;
    const float twoKp = 2.0f * _m.kp, twoKi = 2.0f * _m.ki;

    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) return;
    ax /= n; ay /= n; az /= n;

    float q0 = _m.q.w, q1 = _m.q.x, q2 = _m.q.y, q3 = _m.q.z;

    /* estimated gravity direction, error = cross(measured, estimated) */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    _m.integral[0] += twoKi * ex * dt;
    _m.integral[1] += twoKi * ey * dt;
    _m.integral[2] += twoKi * ez * dt;

    const float d2r = PI_F / 180.0f;   /* dps -> rad/s */
    gx = gx * d2r + twoKp * ex + _m.integral[0];
    gy = gy * d2r + twoKp * ey + _m.integral[1];
    gz = gz * d2r + twoKp * ez + _m.integral[2];

    /* first-order quaternion integration */
    float hdt = 0.5f * dt;
    _m.q.w += hdt * (-q1 * gx - q2 * gy - q3 * gz);
    _m.q.x += hdt * ( q0 * gx + q2 * gz - q3 * gy);
    _m.q.y += hdt * ( q0 * gy - q1 * gz + q3 * gx);
    _m.q.z += hdt * ( q0 * gz + q1 * gy - q2 * gx);

    quatNormalize();
}

bool CarImu::readImu() {
    uint8_t b[6];
    if (!rd(OUTX_L_G, b, 6)) return false;
    _s.gx = i16(&b[0]) * GYRO_LSB;
    _s.gy = i16(&b[2]) * GYRO_LSB;
    _s.gz = i16(&b[4]) * GYRO_LSB;
    if (!rd(OUTX_L_XL, b, 6)) return false;
    _s.ax = i16(&b[0]) * ACCEL_LSB;
    _s.ay = i16(&b[2]) * ACCEL_LSB;
    _s.az = i16(&b[4]) * ACCEL_LSB;

    return true;
}

bool CarImu::begin(const CarTick *tick) {
    _tick = tick;
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    uint8_t id = 0;
    if (!rd(WHO_AM_I, &id, 1) || id != 0x6A) return false;

    wr(CTRL1_XL, 0x40);   /* +-2g   @104Hz */
    wr(CTRL2_G,  0x4C);   /* +-2000dps @104Hz */
    wr(CTRL3_C,  0x44);   /* BDU + IF_INC */
    mahonyInit();

    _lastTick = tick ? tick->count() : 0;
    _lastUs = micros();
    return true;
}

bool CarImu::calibrate(uint16_t ms) {
    float gxs = 0, gys = 0, gzs = 0, axs = 0, ays = 0, azs = 0;
    float maxDev = 0.0f;
    int n = 0;
    unsigned long t0 = millis(), tick = micros();

    while (millis() - t0 < ms) {
        if ((unsigned long)(micros() - tick) >= PERIOD_US) {
            tick = micros();
            if (readImu()) {
                gxs += _s.gx; gys += _s.gy; gzs += _s.gz;
                axs += _s.ax; ays += _s.ay; azs += _s.az;
                n++;
                if (n > 8) {                /* spread vs running mean */
                    float m[3] = {gxs / n, gys / n, gzs / n};
                    float g[3] = {_s.gx, _s.gy, _s.gz};
                    for (int i = 0; i < 3; i++) {
                        float d = fabsf(g[i] - m[i]);
                        if (d > maxDev) maxDev = d;
                    }
                }
            }
        }
    }
    if (n == 0) return false;

    float inv = 1.0f / (float)n;
    float gxm = gxs * inv, gym = gys * inv, gzm = gzs * inv;
    float axm = axs * inv, aym = ays * inv, azm = azs * inv;

    /* accel offset bias is NOT removed here: the fused "up" must stay the real
     * gravity direction. Removing 1 g from the dominant axis (what the
     * reference did) forces "up" onto that axis, i.e. it assumes the board lies
     * flat; with a mounted/tilted board that costs cos(tilt) of every measured
     * turn and drags yaw around. The offset is < 0.05 g = under 3 deg of tilt,
     * which yaw does not care about. */
    _b.gx = gxm; _b.gy = gym; _b.gz = gzm;
    _b.ax = 0.0f; _b.ay = 0.0f; _b.az = 0.0f;

    mahonyInitFromUp(axm, aym, azm);
    _yawDeg = 0.0f;
    {   /* roll/pitch now describe the real mount, not zero */
        float r, p, y;
        quatToEuler(&r, &p, &y);
        _rollDeg = r * RAD2DEG;
        _pitchDeg = p * RAD2DEG;
        _lastYawRad = y;
    }
    _stillDev = maxDev;
    _stillSince = 0;
    _still = true;
    _calibrated = true;
    _lastTick = _tick ? _tick->count() : 0;
    _lastUs = micros();
    return maxDev <= CALIB_STILL_DPS;
}

void CarImu::update() {
    /* cadence: hardware heartbeat when attached, else measured micros().
     * Ticks are only consumed once a whole fusion period elapsed, so a slow
     * loop() does not lose time and a fast one does not oversample. */
    float dt;
    if (_tick) {
        uint32_t elapsed = _tick->elapsed(_lastTick);
        if (elapsed == 0) return;
        dt = (float)elapsed * _tick->dt();
        if (dt < DT_ACC) return;
        _lastTick += elapsed;
    } else {
        unsigned long nowUs = micros();
        dt = (float)(nowUs - _lastUs) * 1e-6f;
        if (dt < DT_ACC) return;
        _lastUs = nowUs;
    }
    if (dt > DT_MAX) return;            /* a stall is not a rotation */
    _dt = dt;
    _m.dt = dt;

    if (!readImu()) return;

    float gx = _s.gx - _b.gx;
    float gy = _s.gy - _b.gy;
    float gz = _s.gz - _b.gz;

    /* still-tracker: null residual zero-rate so yaw holds flat */
    if (fabsf(gx) + fabsf(gy) + fabsf(gz) < STILL_DPS) {
        if (_stillSince == 0) _stillSince = millis();
        if (millis() - _stillSince >= STILL_MS) {
            _b.gx += BIAS_ALPHA * gx;
            _b.gy += BIAS_ALPHA * gy;
            _b.gz += BIAS_ALPHA * gz;
        }
        _still = true;
    } else {
        _stillSince = 0;
        _still = false;
    }

    /* yaw rate = the part of the body rate that spins about the true vertical.
     * World up expressed in body coordinates is the 3rd column of the
     * body->world rotation matrix, so this works for a tilted, side or upside
     * down mount without any extra configuration. */
    {
        const float qw = _m.q.w, qx = _m.q.x, qy = _m.q.y, qz = _m.q.z;
        const float ux = 2.0f * (qx * qz + qw * qy);
        const float uy = 2.0f * (qy * qz - qw * qx);
        const float uz = 1.0f - 2.0f * (qx * qx + qy * qy);
        _yawRateDps = (gx * ux + gy * uy + gz * uz) * (float)IMU_YAW_SIGN;
    }

    mahonyUpdate(_s.ax - _b.ax, _s.ay - _b.ay, _s.az - _b.az, gx, gy, gz);

    float roll, pitch, yaw;
    quatToEuler(&roll, &pitch, &yaw);
    _rollDeg  = roll  * RAD2DEG;
    _pitchDeg = pitch * RAD2DEG;

    /* unwrap for the heading loop: atan2 jumps at +-180 */
    float d = yaw - _lastYawRad;
    while (d >  PI_F) d -= 2.0f * PI_F;
    while (d < -PI_F) d += 2.0f * PI_F;
    _yawDeg += d * RAD2DEG;
    _lastYawRad = yaw;
}

float CarImu::gyroBias(int axis) const {
    if (axis == 0) return _b.gx;
    if (axis == 1) return _b.gy;
    if (axis == 2) return _b.gz;
    return 0.0f;
}

void CarImu::zeroYaw() {
    _yawDeg = 0.0f;                     /* _lastYawRad keeps the unwrap edge */
}
