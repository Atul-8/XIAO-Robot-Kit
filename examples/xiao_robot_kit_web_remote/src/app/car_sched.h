/* SPDX-License-Identifier: MIT */
/* car_sched.h - a 20-line cooperative scheduler.
 *
 * The control loop must never wait for anything, so instead of nesting timers in
 * loop() every job gets a period here and runs when it is due. Jobs are plain
 * functions, they return immediately, and none of them owns another job - that is
 * what "no resource contention" means at this scale: one writer per peripheral,
 * all of them driven from the same thread in a fixed order.
 */
#ifndef CAR_SCHED_H
#define CAR_SCHED_H

#include <Arduino.h>

typedef void (*CarTaskFn)(void);

struct CarTask {
    uint32_t  periodMs;   /* 0 = run on every pass */
    uint32_t  nextMs;     /* internal, start at 0 */
    CarTaskFn fn;
};

/* Runs every task whose deadline has passed. Never blocks. */
void schedRun(CarTask *tasks, uint8_t n, uint32_t now);

#endif /* CAR_SCHED_H */