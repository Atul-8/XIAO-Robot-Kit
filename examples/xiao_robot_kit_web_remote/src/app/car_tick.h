/* SPDX-License-Identifier: MIT */
/**
 * @file  car_tick.h
 * @brief Hardware-timer heartbeat (ESP32 timer, 2 ms default).
 *
 * The car does three things at different rates - IMU fusion, encoder speed,
 * control - while also serving HTTP/WebSocket. Pacing all of that off loop()
 * makes the rates drift with Wi-Fi load, so the cadence comes from a hardware
 * timer instead: the ISR only increments a counter, and each module derives its
 * own dt from that counter. That keeps the timing exact even when a request
 * takes 30 ms, and it costs one counter increment per tick.
 *
 * The ISR deliberately does NOT touch I2C: Wire takes a mutex on ESP32, which is
 * illegal in interrupt context. Sampling stays in loop(), only the clock is
 * hardware.
 */
#ifndef CAR_TICK_H
#define CAR_TICK_H

#include <Arduino.h>

class CarTick {
public:
    /** Start the hardware timer. `hz` is rounded to whole microseconds. */
    bool begin(uint32_t hz = 500);

    float    dt() const { return _dt; }          /* seconds per tick */

    /** Monotonic tick counter; wrap-safe when compared with subtraction. */
    uint32_t count() const;

    /** Ticks elapsed since `last`, without touching it. */
    uint32_t elapsed(uint32_t last) const;

private:
    uint32_t _hz = 500;
    float    _dt = 0.002f;
};

#endif /* CAR_TICK_H */
