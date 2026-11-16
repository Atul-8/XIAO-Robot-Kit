/* SPDX-License-Identifier: MIT */
/* car_hw.h - the hardware objects, owned by car_hw.cpp and shared read-mostly.
 *
 * One instance each, created once at static-init time, never copied. Every other
 * module includes this header instead of redeclaring globals: that is what keeps
 * "who owns the motor object" from becoming a guess.
 */
#ifndef CAR_HW_H
#define CAR_HW_H

#include <Arduino.h>
#include "src/app/car_tick.h"
#include "src/hw/car_motor.h"
#include "src/hw/car_encoder.h"
#include "src/hw/car_imu.h"
#include "src/ctl/car_control.h"
#include "src/net/car_web.h"

extern CarTick    tick;      /* hardware heartbeat, 500 Hz: the exact clock */
extern CarMotor   motor;     /* M1 IN1/IN2 = D1/D2, M2 IN1/IN2 = D19/D18 */
extern CarEncoder encoder;   /* M1 A/B = D3/D14, M2 A/B = D17/D10 */
extern CarImu     imu;       /* I2C SDA/SCL = D4/D5 */
extern CarControl ctl;       /* speed PI x2 inside the yaw loop */
extern CarWeb     web;       /* HTTP page + WebSocket */

/* Pins, heartbeat, encoders, control wiring. Must run before anything drives a
 * motor or reads a sensor, and before ctl.<anything> is called. */
void hwBegin(void);

/* Drive in LEFT/RIGHT terms from the bench and the console, i.e. through the same
 * MOTOR_SWAP_LR seam the control loop uses, so a test cannot disagree with the
 * car. */
void hwSetWheels(int leftDuty, int rightDuty);

/* IMU bring-up only (bus scan + configure). Kept separate because the caller has
 * to decide what to do when it fails: the car still drives without it. */
bool hwImuBegin(void);

#endif /* CAR_HW_H */