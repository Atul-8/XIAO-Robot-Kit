/* SPDX-License-Identifier: MIT */
/* car_log.cpp - serial + page-mirror output.
 *
 * Note that `Serial` is false on a XIAO ESP32-S3 when no USB host is attached:
 * a `if (!Serial) return;` here would silently drop every message, including the
 * ones a phone is supposed to see. The gate is therefore per sink, not global.
 */
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "src/app/car_config.h"
#include "src/hw/car_hw.h"
#include "src/app/car_log.h"

static CarWeb *s_web = nullptr;

void logBind(CarWeb *w) { s_web = w; }

static void emitLine(const char *buf) {
    if (Serial) Serial.println(buf);
    if (s_web) {
        char fr[200];                  /* the page frame drops the tail of a long line */
        snprintf(fr, sizeof(fr), "L:%s", buf);
        s_web->send(fr);
    }
}

void say(const char *fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    emitLine(buf);
    /* The USB CDC write can block once its buffer fills if no host drains it.
     * One yield keeps the heartbeat and Wi-Fi alive behind a chatty log. */
    yield();
}

void reply(const char *what) {
    char buf[48];
    snprintf(buf, sizeof(buf), "OK %s", what);
    if (s_web) s_web->send(buf);
}

void logCommand(const char *tag, const char *msg) {
    say("[%8lu] %-4s \"%.24s\" mode=%-4s mmps=%+.0f hdg=%d yaw=%+.1f tyaw=%+.1f "
        "ydot=%+.0f trn=%+d duty=%d/%d",
        (unsigned long)millis(), tag, msg,
        ctl.modeName(), (double)ctl.targetMmps(), ctl.headingHold() ? 1 : 0,
        (double)imu.yawDeg(), (double)ctl.targetYaw(), (double)imu.yawRateDps(),
        ctl.turnDir(), ctl.dutyL(), ctl.dutyR());
}