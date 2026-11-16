/* SPDX-License-Identifier: MIT */
/**
 * @file  car_motor.cpp
 * @brief Dual H-bridge: 20 kHz / 10-bit PWM, three-state drive, dead zone.
 */

#include "src/hw/car_motor.h"

#define DEAD_ZONE 15      /* |duty| below this cannot move the motor */

CarMotor::CarMotor(uint8_t l1, uint8_t l2, uint8_t r1, uint8_t r2)
    : _l1(l1), _l2(l2), _r1(r1), _r2(r2) {}

void CarMotor::begin() {
    /* outputs low BEFORE the PWM peripheral takes them over (no boot glitch) */
    const uint8_t pins[4] = {_l1, _l2, _r1, _r2};
    for (int i = 0; i < 4; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
        ledcAttach(pins[i], MOTOR_PWM_HZ, MOTOR_PWM_RES);
    }
    coast();
}

void CarMotor::apply(uint8_t in1, uint8_t in2, int duty) {
    if (duty >  100) duty =  100;
    if (duty < -100) duty = -100;
    if (duty > -DEAD_ZONE && duty < DEAD_ZONE) duty = 0;

    uint32_t pwm = (uint32_t)((duty < 0 ? -duty : duty) * 1023 / 100);
    if (duty > 0)      { ledcWrite(in1, pwm); ledcWrite(in2, 0); }        /* forward */
    else if (duty < 0) { ledcWrite(in1, 0);   ledcWrite(in2, pwm); }      /* reverse */
    else               { ledcWrite(in1, 0);   ledcWrite(in2, 0); }        /* coast   */
}

void CarMotor::set(int leftDuty, int rightDuty) {
    _left  = leftDuty  * _sgnL;
    _right = rightDuty * _sgnR;
    apply(_l1, _l2, _left);
    apply(_r1, _r2, _right);
}
