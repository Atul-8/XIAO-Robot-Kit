/* SPDX-License-Identifier: MIT */
/**
 * @file  car_encoder.h
 * @brief Two quadrature encoders, x4 decoding, speed and distance.
 *
 * Pins (board-authoritative):
 *   M1 A = D3  (GPIO4)   M1 B = D14 (GPIO41)
 *   M2 A = D17 (GPIO13)  M2 B = D10 (GPIO9)
 *
 * 7200 counts per output revolution = 12 PPR x 4 (quadrature) x 150 (gearbox).
 *
 * The ISRs only decode and add; everything else runs in update(). Speed is
 * measured over a fixed window, timed by the hardware heartbeat, so it does not
 * drift when loop() is busy with Wi-Fi (millis() is the fallback).
 * ENC_L_SIGN / ENC_R_SIGN flip a channel whose A/B pair is wired the other way,
 * and they also define which physical direction counts as "forward".
 */
#ifndef CAR_ENCODER_H
#define CAR_ENCODER_H

#include <Arduino.h>
#include "src/app/car_tick.h"

/*
 * "Forward" is defined by the ENCODER sign, not by the motor sign: the wheel
 * direction that makes a count go UP is called forward. Everything downstream
 * (rpm, mm, the speed PI) then agrees with the duty. Done on the motor side
 * instead, the PI would get a negative plant gain and run away.
 */
#ifndef ENC_L_SIGN
#define ENC_L_SIGN (-1)     /* M1 (D3/D14) */
#endif
#ifndef ENC_R_SIGN
#define ENC_R_SIGN (-1)     /* M2 (D17/D10) */
#endif
/* Both stay -1: on the real chassis "forward" already pointed the right way.
 * The car only LOOKED like it was driving backwards because ONE channel's
 * polarity was wrong, so the two wheels span opposite ways - a single-channel
 * fix (serial key 1), not a convention flip. Do not touch this pair to cure a
 * one-wheel symptom; see car_motor.h. */

#define ENC_CPR        7200.0f    /* counts per output revolution */
#define ENC_WINDOW_MS  50         /* speed measurement window */

/* One encoder channel: the ISR owns count/state, the class reads them. */
struct EncChan {
    uint8_t a = 0, b = 0;
    int8_t  sign = 1;
    volatile int32_t count = 0;
    volatile uint8_t state = 0;
};

class CarEncoder {
public:
    CarEncoder(uint8_t lA, uint8_t lB, uint8_t rA, uint8_t rB);

    /** Attach the ISRs. `tick` may be null (millis fallback). */
    bool begin(const CarTick *tick = nullptr);

    int32_t left() const;             /* cumulative counts, sign-corrected */
    int32_t right() const;

    /** Refresh rpmL()/rpmR() once per window. Call from the loop. */
    void update();

    float rpmL() const { return _rpmL; }
    float rpmR() const { return _rpmR; }

    /** Distance travelled by the wheel pair, mm (positive = forward). */
    float mm() const;

    void setWheelDiameter(float mm) { _wheelDia = mm; }

private:
    float countsToRpm(int32_t delta, float dt) const;

    EncChan _l, _r;
    float _rpmL = 0.0f, _rpmR = 0.0f;
    int32_t _lastL = 0, _lastR = 0;
    float _counts = 0.0f;             /* averaged both wheels, for distance */
    float _wheelDia = 50.0f;          /* mm */
    const CarTick *_tick = nullptr;
    uint32_t _lastTick = 0;
    unsigned long _windowAt = 0;
};

#endif /* CAR_ENCODER_H */
