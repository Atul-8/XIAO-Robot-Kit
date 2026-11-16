/* SPDX-License-Identifier: MIT */
/* car_log.h - the single output path of the sketch.
 *
 * Every diagnostic line goes through here and reaches BOTH sinks: the USB serial
 * port and the page's log window. There is no debug switch on purpose - the log
 * is how the page stays usable without a cable, so it is not optional equipment.
 */
#ifndef CAR_LOG_H
#define CAR_LOG_H

#include <Arduino.h>
#include "src/net/car_web.h"

/* Bind the web link used for the page mirror. Pass nullptr to disable it. */
void logBind(CarWeb *w);

/* Formatted line: USB serial + page log window. */
void say(const char *fmt, ...);

/* Short "OK ..." acknowledgement back to the page. */
void reply(const char *what);

/* One line per command, printed AFTER it has been applied so the numbers in it
 * are the state the car is about to act on. */
void logCommand(const char *tag, const char *msg);

#endif /* CAR_LOG_H */