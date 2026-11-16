/* SPDX-License-Identifier: MIT */
/**
 * @file  car_motor.h
 * @brief Dual H-bridge driver (FACTS three-state + dead zone + safe power-on).
 *
 * Pins (board-authoritative):
 *   M1 IN1 = D1  (GPIO2)   M1 IN2 = D2  (GPIO3)
 *   M2 IN1 = D19 (GPIO11)  M2 IN2 = D18 (GPIO12)
 *
 * duty: -100..100. |duty| below DEAD_ZONE coasts (the motor has no torque
 * there), so a closed loop must be given enough authority to leave the zone.
 *
 * Each channel's forward direction is defined on its own, because the two sides
 * are wired mirrored: the same duty makes one wheel go forward and the other
 * backward. The polarity comes from MOTOR_x_INV, a compile-time constant; there
 * is no runtime setter, so a flashed board cannot drift out of configuration.
 */
#ifndef CAR_MOTOR_H
#define CAR_MOTOR_H

#include <Arduino.h>

/* ---- the ONLY three direction knobs on this robot --------------------------
 * Every chassis is wired slightly differently; these three integers are the whole
 * configuration. Nothing is measured, learned or auto-corrected at runtime, so a
 * fresh flash always behaves exactly like the last one.
 *
 *   1) MOTOR_SWAP_LR  1 = M1 is the physically RIGHT wheel, so duty and encoder
 *                     routing are swapped. This is also what decides which way
 *                     "turn left" goes, because a pivot only drives one wheel.
 *   2) MOTOR_L_INV    1 = flip M1, i.e. '+duty' spins it backwards
 *   3) MOTOR_R_INV    1 = flip M2, same rule
 *
 * Measured on this chassis: 1 / 0 / 1.
 * Symptom -> knob:
 *   car drives BACKWARD on forward  -> flip 2) AND 3) together
 *   only ONE wheel spins backwards  -> flip that one knob
 *   arrows turn the car the wrong way, or it veers away from its heading
 *                                   -> flip 1)
 * The two motor polarities must always be OPPOSITE on a mirrored chassis like
 * this one; equal values mean one wheel fights the other. ------------------ */
#ifndef MOTOR_SWAP_LR
#define MOTOR_SWAP_LR  1    /* M1 is the RIGHT wheel */
#endif
#ifndef MOTOR_L_INV
#define MOTOR_L_INV    0    /* M1: '+duty' is already forward */
#endif
#ifndef MOTOR_R_INV
#define MOTOR_R_INV    1    /* M2: mounted mirrored, so it needs the flip */
#endif

#define MOTOR_L_SIGN   (MOTOR_L_INV ? -1 : +1)
#define MOTOR_R_SIGN   (MOTOR_R_INV ? -1 : +1)

#define MOTOR_PWM_HZ  20000
#define MOTOR_PWM_RES 10

class CarMotor {
public:
    CarMotor(uint8_t l1, uint8_t l2, uint8_t r1, uint8_t r2);

    /** Outputs low, attach PWM, park. Safe to call before anything else. */
    void begin();

    /** -100..100 per CHANNEL, i.e. (M1, M2) - not logical left/right: that
     *  mapping lives in car_hw.cpp, so there is exactly one place it happens.
     *  Polarity and dead zone are applied inside. */
    void set(int m1Duty, int m2Duty);

    /** Both sides coast. */
    void coast() { set(0, 0); }

    int left() const { return _left; }     /* what was really driven (after sign) */
    int right() const { return _right; }

private:
    void apply(uint8_t in1, uint8_t in2, int duty);

    uint8_t _l1, _l2, _r1, _r2;
    const int8_t _sgnL = MOTOR_L_SIGN;
    const int8_t _sgnR = MOTOR_R_SIGN;
    int _left = 0, _right = 0;
};

#endif /* CAR_MOTOR_H */
