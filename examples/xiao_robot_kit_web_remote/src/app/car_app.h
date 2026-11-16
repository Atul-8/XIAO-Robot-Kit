/* SPDX-License-Identifier: MIT */
/* car_app.h - what the .ino is allowed to call: begin once, step forever. */
#ifndef CAR_APP_H
#define CAR_APP_H

#include <Arduino.h>

/* Ordered boot: hardware, IMU calibration, battery report, Wi-Fi. Blocks on purpose -
 * nothing may move while the gyro is being calibrated. */
void appBegin(void);

/* One pass of every due job. Never blocks: see the task table in car_app.cpp. */
void appStep(void);

/* Re-zero calibration (gyro bias + gravity seed + yaw 0 + heading target 0).
 * Shared by serial `c` and the page's CALIBRATE key so the two can never drift
 * apart. Blocks for CALIB_MS on purpose: the bias needs an uninterrupted still
 * window. It stops the motors first and tells the stall watchdog afterwards that
 * the gap was intended. Returns false if the board was moving. */
bool appCalibrate(void);

/* "That gap was on purpose": resets the loop-stall watchdog baseline. */
void appQuiet(void);

#endif /* CAR_APP_H */