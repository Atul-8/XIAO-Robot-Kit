/* SPDX-License-Identifier: MIT */
/* car_console.h - the USB-serial bench console (keys, self test, diagnosis).
 *
 * Everything here is allowed to block: it is driven by a human on a bench and it
 * pumps the IMU and the encoders while it waits. `consoleBusy()` tells the main
 * loop that a long bench routine is running, so the loop-stall watchdog does not
 * report it as a fault.
 */
#ifndef CAR_CONSOLE_H
#define CAR_CONSOLE_H

#include <Arduino.h>

void consolePoll(void);            /* non-blocking key scan */
void consoleHelp(void);
void consoleBattery(void);         /* `b`: ADC + divider cross-check */
void consoleState(void);           /* one STATE line: why a turn did nothing */

/* `t`: drives both wheels then one wheel, prints measured vs expected plus a
 * verdict line. Blocking (~6 s) and it moves the car. */
void consoleSelfTest(void);

/* 1 while a blocking bench routine is driving the wheels. */
bool consoleBusy(void);

#endif /* CAR_CONSOLE_H */