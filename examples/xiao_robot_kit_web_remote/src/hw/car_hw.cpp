/* SPDX-License-Identifier: MIT */
/* car_hw.cpp - owns the six hardware objects and the control<->hardware glue.
 *
 * This is the only file that knows which pin belongs to which channel; everything
 * above it talks to objects, never to GPIO numbers.
 */
#include <Arduino.h>
#include "src/app/car_config.h"
#include "src/hw/car_battery.h"
#include "src/hw/car_hw.h"

CarTick    tick;
CarMotor   motor(2, 3, 11, 12);            /* M1 D1/D2, M2 D19/D18 */
CarEncoder encoder(4, 41, 13, 9);          /* M1 D3/D14, M2 D17/D10 */
CarImu     imu;
CarControl ctl;
CarWeb     web;

/* -- control <-> hardware glue: the only place the two sides meet --
 * MOTOR_SWAP_LR is applied HERE and nowhere else, so a chassis whose motors are
 * wired the other way round is fixed by one integer - and duty and encoder speed
 * are always swapped together, which is what keeps a speed loop from regulating
 * the opposite wheel. */
#if MOTOR_SWAP_LR
static void ioSetDuty(int l, int r) { motor.set(r, l); }
static float ioRpmL(void)    { return encoder.rpmR(); }
static float ioRpmR(void)    { return encoder.rpmL(); }
#else
static void ioSetDuty(int l, int r) { motor.set(l, r); }
static float ioRpmL(void)    { return encoder.rpmL(); }
static float ioRpmR(void)    { return encoder.rpmR(); }
#endif

void hwSetWheels(int leftDuty, int rightDuty) { ioSetDuty(leftDuty, rightDuty); }
static float ioYawDeg(void)  { return imu.yawDeg(); }
static float ioYawRate(void) { return imu.yawRateDps(); }
static float ioMm(void)      { return encoder.mm(); }

/* The yaw loop may only run on a calibrated sensor, otherwise its own noise is
 * amplified into a permanent steering offset. */
static bool ioYawValid(void) { return imu.calibrated(); }

void hwBegin(void) {
    tick.begin(500);                       /* 2 ms heartbeat: exact dt */
    motor.begin();
    encoder.begin(&tick);
    encoder.setWheelDiameter(WHEEL_DIA_MM);
    batteryBegin();                        /* resolution + attenuation + pad pull */

    const CarControlIo io{ioSetDuty, ioRpmL, ioRpmR, ioYawDeg, ioYawRate, ioYawValid, ioMm};
    ctl.begin(io, &tick);
}

bool hwImuBegin(void) { return imu.begin(&tick); }