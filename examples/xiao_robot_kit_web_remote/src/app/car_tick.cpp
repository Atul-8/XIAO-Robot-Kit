/* SPDX-License-Identifier: MIT */
/**
 * @file  car_tick.cpp
 * @brief ESP32 hardware timer used as the system heartbeat.
 */

#include "src/app/car_tick.h"
#include <esp32-hal-timer.h>

static volatile uint32_t s_ticks = 0;
static hw_timer_t *s_timer = nullptr;

static void IRAM_ATTR onTimer() { s_ticks = s_ticks + 1; }   /* no ++ on volatile */

bool CarTick::begin(uint32_t hz) {
    if (hz < 50) hz = 50;
    if (hz > 10000) hz = 10000;
    _hz = hz;
    _dt = 1.0f / (float)hz;

    s_timer = timerBegin(1000000);          /* 1 MHz: alarms are microseconds */
    if (!s_timer) return false;
    timerAttachInterrupt(s_timer, onTimer);
    timerAlarm(s_timer, 1000000u / hz, true, 0);
    s_ticks = 0;
    return true;
}

uint32_t CarTick::count() const { return s_ticks; }

uint32_t CarTick::elapsed(uint32_t last) const { return s_ticks - last; }
