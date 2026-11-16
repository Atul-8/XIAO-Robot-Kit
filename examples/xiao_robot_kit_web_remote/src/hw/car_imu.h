/* SPDX-License-Identifier: MIT */
/**
 * @file  car_imu.h
 * @brief LSM6DS3TR-C + Mahony AHRS -> roll / pitch / yaw in degrees.
 *
 * This is the 06_IMU_Euler algorithm moved into a module, unchanged:
 *   - Mahony kp = 2.0, ki = 0, error = cross(a_measured, a_estimated),
 *     first-order quaternion integration, then normalise
 *   - quaternion -> ZYX euler (roll / pitch / yaw)
 *   - boot calibration: gyro bias = mean; the accel mean is the gravity
 *     direction and is used to seed the attitude, so the board does NOT have to
 *     be mounted flat (the reference subtracted 1 g from the dominant axis,
 *     which silently assumed a level board)
 *   - still-tracker: while parked, leak the residual zero-rate into the bias so
 *     yaw holds flat instead of creeping
 *
 * Two additions needed by the car (nothing in the fusion itself is changed):
 *   - the integration step comes from the hardware-timer heartbeat (exact dt),
 *     with a micros() fallback when no heartbeat is attached
 *   - yaw is additionally kept unwrapped (continuous degrees) for the heading
 *     loop, because atan2 jumps at +-180 and a heading error must not jump
 *
 * yaw is RELATIVE (no magnetometer): it can hold and turn by an angle, it can
 * never point north.
 */
#ifndef CAR_IMU_H
#define CAR_IMU_H

#include <Arduino.h>
#include "src/app/car_tick.h"

/* Compile-time mounting fix: -1 when the car turns left but yawDeg shrinks. */
#ifndef IMU_YAW_SIGN
#define IMU_YAW_SIGN 1
#endif

struct Quat   { float w, x, y, z; };
struct Mahony { Quat q; float kp, ki, dt; float integral[3]; };
struct ImuSample { float gx, gy, gz, ax, ay, az; };
struct ImuBias   { float gx, gy, gz, ax, ay, az; };

class CarImu {
public:
    /** I2C + registers + filter reset. `tick` may be null (micros fallback). */
    bool begin(const CarTick *tick = nullptr);

    /** Blocking boot calibration (`ms` of samples). The board must be STILL but
     *  not level: whatever attitude it is mounted in becomes the reference
     *  (yaw then counts rotation about the true vertical). The return value says
     *  whether the board actually was still, so the caller can retry. */
    bool calibrate(uint16_t ms);

    /** Timer-paced: samples and integrates when a new heartbeat arrived. */
    void update();

    float rollDeg()  const { return _rollDeg; }
    float pitchDeg() const { return _pitchDeg; }
    float yawDeg()   const { return _yawDeg * (float)IMU_YAW_SIGN; }  /* unwrapped */
    float yawRawDeg() const { return _lastYawRad * 57.29578f; }  /* wrapped, quat */
    /** Rotation rate about the TRUE vertical, deg/s, + = counter-clockwise.
     *  Body rate projected on world-up, so it is correct for any mounting
     *  angle (and much cleaner than differentiating yaw). */
    float yawRateDps() const { return _yawRateDps; }
    float stillDevDps() const { return _stillDev; }            /* last calibration */

    bool calibrated() const { return _calibrated; }
    bool still() const      { return _still; }

    float gyroBias(int axis) const;

    void   zeroYaw();

private:
    bool readImu();
    void quatIdentity();
    void quatNormalize();
    void quatToEuler(float *roll, float *pitch, float *yaw);
    void mahonyInit();
    void mahonyInitFromUp(float ux, float uy, float uz);
    void mahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz);

    Mahony    _m;
    ImuBias   _b;
    ImuSample _s;

    float _rollDeg = 0.0f, _pitchDeg = 0.0f;
    float _yawDeg = 0.0f;          /* unwrapped */
    float _yawRateDps = 0.0f;      /* gyro about the vertical, deg/s */
    float _lastYawRad = 0.0f;      /* wrapped, for unwrapping */
    float _dt = 0.0f;
    float _stillDev = 0.0f;

    bool _calibrated = false;
    bool _still = false;

    const CarTick *_tick = nullptr;
    uint32_t _lastTick = 0;
    unsigned long _lastUs = 0;
    unsigned long _stillSince = 0;
};

#endif /* CAR_IMU_H */
