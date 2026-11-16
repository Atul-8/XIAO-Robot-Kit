/* SPDX-License-Identifier: MIT */
/* xiao_robot_kit_web_remote - web-remote car: encoder speed loops + gyro differential.
 *
 * The ESP32 hosts the control page and a WebSocket server on one TCP port, so a
 * phone browser drives the car with no host app: http://xiaorobot.local
 *
 * src/app  car_app     boot, command router, telemetry, task table
 *          car_config  every tunable: Wi-Fi, periods, battery divider
 *          car_sched   cooperative task table: period, deadline, job
 *          car_tick    hardware timer heartbeat, exact dt for every control step
 *          car_log     the output path: USB serial + the page's log window
 *          car_console serial keys, bench self test, battery/network diagnosis
 * src/ctl  car_control motion state machine: wheel speed PI x2 inside a yaw loop
 * src/hw   car_hw      owns the objects + the one place MOTOR_SWAP_LR applies
 *          car_motor   H-bridge, dead zone, per-channel polarity, safe power-on
 *          car_encoder two x4 quadrature encoders, windowed rpm, distance
 *          car_imu     Mahony AHRS, calibration, drift compensation
 *          car_battery ADC divider -> tap mV -> volts -> rough state of charge
 * src/net  car_web     HTTP page + WebSocket protocol + telemetry (never blocks)
 *
 * Wiring: M1 IN1/IN2 = D1/D2, M2 IN1/IN2 = D19/D18, battery = D16,
 *         M1 enc A/B = D3/D14, M2 enc A/B = D17/D10, IMU SDA/SCL = D4/D5.
 *
 * Zero third-party libraries.
 */
#include <Arduino.h>
#include "src/app/car_app.h"

void setup() { appBegin(); }

void loop() { appStep(); }