/* SPDX-License-Identifier: MIT */
/* car_sched.cpp - see car_sched.h. */
#include <Arduino.h>
#include "src/app/car_sched.h"

void schedRun(CarTask *tasks, uint8_t n, uint32_t now) {
    for (uint8_t i = 0; i < n; i++) {
        CarTask &t = tasks[i];
        /* signed compare: correct across the millis() wrap */
        if ((int32_t)(now - t.nextMs) < 0) continue;
        t.nextMs = now + t.periodMs;
        t.fn();
    }
}