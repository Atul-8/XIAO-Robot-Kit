/* SPDX-License-Identifier: MIT */
/* car_app.cpp - boot sequence, command router, telemetry, the task table.
 *
 * Boot is a straight line (hardware -> IMU calibration -> battery report -> page)
 * and then everything periodic is a job in kTasks: one writer per peripheral, no
 * job starts another job, nothing waits. That is the whole concurrency model.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "src/app/car_config.h"
#include "src/hw/car_hw.h"
#include "src/app/car_log.h"
#include "src/hw/car_battery.h"
#include "src/app/car_console.h"
#include "src/app/car_sched.h"
#include "src/app/car_app.h"

/* Baseline for the loop-stall watchdog. File scope so appQuiet() can move it after
 * an intentional blocking routine (calibration) instead of reporting it as a stall. */
static uint32_t s_lastStep = 0;
static bool s_calPending = false;   /* the page asked for a calibration */

/* ── command router: one entry point, one reply, one optional log line ── */
static void onCommand(const char *msg) {
    /* printed once, before the per-command log: proves the command path works
     * end to end the first time the page talks to the car */
    static bool s_first = true;
    if (s_first) {
        s_first = false;
        say("[web] first command received: \"%s\"", msg);
    }

    float v = 0.0f;
    int i = 0;
    const char *tag = "ERR";

    if (sscanf(msg, "DRV:%f,%d", &v, &i) == 2) {
        /* the command the page actually uses: speed AND turn in one message,
         * so driving and turning can never cancel each other */
        ctl.setDrive(v, i);
        reply("DRV");
        tag = "DRV";
    } else if (sscanf(msg, "SPD:%f", &v) == 1) {
        ctl.setSpeed(v);
        reply("SPD");
        tag = "SPD";
    } else if (sscanf(msg, "PIVOT:%d", &i) == 1) {
        ctl.pivot(i);
        reply(i ? "PIVOT" : "STOP");
        tag = "PIVOT";
    } else if (sscanf(msg, "TURN:%f", &v) == 1) {
        ctl.turnBy(v);
        reply("TURN");
        tag = "TURN";
    } else if (sscanf(msg, "HDG:%d", &i) == 1) {
        ctl.holdHeading(i != 0);
        reply(i ? "HDG on (gyro differential)" : "HDG off (plain speed loops)");
        tag = "HDG";
    } else if (strcmp(msg, "FWD") == 0) {
        /* "set forward": whatever the car points at now becomes yaw 0, so the next
         * TURN is measured from here */
        imu.zeroYaw();
        ctl.zeroHeading();
        reply("FWD");
        tag = "FWD";
    } else if (strcmp(msg, "CAL") == 0) {
        /* Only flag it here: calibrating blocks for CALIB_MS and this is the middle
         * of the WebSocket receive path. appStep() runs it at a clean loop point. */
        ctl.stop();
        s_calPending = true;
        reply("CAL keep the car still");
        tag = "CAL";
    } else if (strcmp(msg, "STOP") == 0) {
        ctl.stop();
        reply("STOP");
        tag = "STOP";
    } else {
        web.send("ERR");
    }

    logCommand(tag, msg);
}

static void sendTelemetry(void) {
    char buf[96];
    /* mm, battery ADC counts, battery tap mV, wheel rpm L/R, calibrated.
     * Both battery numbers are raw: counts are what the ADC produced, tap mV is what
     * a meter on the pad reads, and the page turns mV into volts with the divider
     * ratio. Nothing here depends on a calibrated "battery model". */
    snprintf(buf, sizeof(buf), "T:%.0f,%d,%.0f,%.0f,%.0f,%d",
             (double)encoder.mm(), batteryAdc(), (double)batteryPinMv(),
             (double)encoder.rpmL(), (double)encoder.rpmR(),
             imu.calibrated() ? 1 : 0);
    web.send(buf);
}

/* ── jobs ──
 * Order matters only in that sensor sampling comes before the control step that
 * uses it; each job is independent and returns immediately. */
static void jobImu(void)      { imu.update(); }       /* self-paced at ODR_HZ */
static void jobEncoder(void)  { encoder.update(); }   /* self-paced, 50 ms window */
static void jobControl(void)  { ctl.update(); }       /* self-paced at CTRL_MS */
static void jobWeb(void)      { web.poll(); }         /* accept + parse, no waits */
static void jobConsole(void)  { consolePoll(); }
static void jobTelemetry(void){ sendTelemetry(); }

/* Says out loud that the radio is up but nobody ever opened the page: without it
 * "the car does not move" and "the phone never connected" look identical. Re-armed
 * every time the link drops, so a reconnect prints it again. */
static void jobLink(void) {
    static bool told = false;
    if (!web.wifiUp()) { told = false; return; }
    if (told) return;
    told = true;
    if (web.connected()) return;
    say("[web] Wi-Fi is up but no page connected yet -> open http://" HOSTNAME ".local");
}

static void jobState(void) {
    if (ctl.mode() != CAR_IDLE) consoleState();       /* quiet while parked */
}

static CarTask kTasks[] = {
    /*  period            next  job */
    {        0,            0,   jobImu       },
    {        0,            0,   jobEncoder   },
    {        0,            0,   jobControl   },
    {        0,            0,   jobWeb       },
    {  CONSOLE_MS,         0,   jobConsole   },
    {  TELEMETRY_MS,       0,   jobTelemetry },
    {  5000,               0,   jobLink       },
    {  STATE_MS,           0,   jobState     },
};
static const uint8_t kTaskCount = (uint8_t)(sizeof(kTasks) / sizeof(kTasks[0]));

/* ── boot steps ── */
static bool bootImu(void) {
    const bool onBus = hwImuBegin();
    if (!onBus) return false;
    /* Calibration needs a STILL car. Anything that drives a wheel before this
     * point makes it fail (cal=0), which silently disables heading hold and the
     * pivot rate loop - turns then look dead. That is why nothing moves at boot. */
    bool ok = imu.calibrate(CALIB_MS);
    if (!ok) ok = imu.calibrate(CALIB_MS);            /* one retry */
    if (Serial) {
        Serial.print(ok ? "IMU calibration done, gyro bias " : "IMU calibration FAILED, bias ");
        Serial.print(imu.gyroBias(0), 2); Serial.print(" / ");
        Serial.print(imu.gyroBias(1), 2); Serial.print(" / ");
        Serial.print(imu.gyroBias(2), 2); Serial.print(" dps");
        /* a big zero-rate offset is normal for this part, a huge one means the car
         * was nudged while calibrating (or an axis is dead) */
        const float big = fmaxf(fmaxf(fabsf(imu.gyroBias(0)), fabsf(imu.gyroBias(1))),
                                fabsf(imu.gyroBias(2)));
        Serial.println(big > 10.0f ? "  (large - was the car moving?)" : "");
    }
    return imu.calibrated();
}

static void bootZero(void) {
    /* The three direction macros are the whole configuration - nothing is probed
     * or learned at run time, so a fresh flash drives exactly like the last one.
     * Printing them means a pasted boot log always says how the car was told to
     * move. Yaw then starts from zero whatever happened during calibration. */
    say("[mot] direction: swapLR=%d M1 inv=%d M2 inv=%d (car_motor.h)",
        MOTOR_SWAP_LR, MOTOR_L_INV, MOTOR_R_INV);
    /* Battery diagnosis on EVERY boot, not behind the `b` key: "why does it show
     * 8.5 V" took three round trips because the numbers lived behind a keypress
     * nobody remembered. Four lines at boot cost nothing and answer it. */
    consoleBattery();
    imu.zeroYaw();
}

void appQuiet(void) { s_lastStep = millis(); }

bool appCalibrate(void) {
    ctl.stop();
    say("[cal] keep the car still for %d ms...", CALIB_MS);
    bool ok = imu.calibrate(CALIB_MS);
    if (!ok) ok = imu.calibrate(CALIB_MS);            /* one retry */
    ctl.zeroHeading();
    say("[cal] ok=%d dev=%.2f dps  gyro bias %.2f/%.2f/%.2f dps  roll %+.1f pitch %+.1f  "
        "yaw=0  hdg target=0",
        ok ? 1 : 0, (double)imu.stillDevDps(), (double)imu.gyroBias(0),
        (double)imu.gyroBias(1), (double)imu.gyroBias(2),
        (double)imu.rollDeg(), (double)imu.pitchDeg());
    if (!ok) say("[cal] the board was moving, try again");
    appQuiet();                                       /* the gap was on purpose */
    return ok;
}

static void bootWeb(void) {
    web.onCommand(onCommand);
    /* begin() only STARTS the radio now: it used to wait up to 15 s for the link,
     * which kept the wheels dead and the console silent for the whole retry dance.
     * The link-up, the IP and the "open this URL" hint are reported by poll(). */
    web.begin(WIFI_SSID, WIFI_PASS, HOSTNAME, HTTP_PORT);
    say("Wi-Fi: joining \"%s\" in the background - the car stays controllable on the serial console", WIFI_SSID);
    if (Serial) {
        Serial.println("Commands: DRV:<mm/s>,<turn> | SPD | PIVOT | TURN | HDG | FWD | STOP");
        consoleHelp();
    }
}

void appBegin(void) {
    Serial.begin(115200);
    delay(200);       /* boot, once: let USB CDC enumerate or the banner is lost */
    /* A build stamp on purpose: "did my flash actually take?" used to be
     * unanswerable, because every build printed the same banner. */
    if (Serial) {
        Serial.print("11 WebRemote - encoder speed loops + gyro heading | build ");
        Serial.print(__DATE__);
        Serial.print(" ");
        Serial.println(__TIME__);
    }

    hwBegin();                 /* pins, heartbeat, encoders, control wiring */
    logBind(&web);             /* from here on every line also reaches the page */

    const bool imuOk = bootImu();
    if (!imuOk)
        say("WARN IMU NOT CALIBRATED -> no heading hold and turns use the open-loop "
            "fallback; hold the car still and press c");

    bootZero();                /* direction macros + battery report + yaw 0 */
    bootWeb();

    /* Boot is over, so the serial port stops being allowed to stall the car.
     *
     * HWCDC::write() waits up to 20 x tx_timeout_ms = 2 s when a host is attached
     * but not reading (a paused serial monitor, a terminal that lost its window,
     * a tool that opened the port and went quiet). Measured on this board: one
     * log line froze the loop for 2.0 s, two lines for 3.5 s of missing
     * telemetry. With the timeout at 0 a full ring costs a short write instead,
     * i.e. the serial log becomes best-effort and the page mirror stays complete
     * (it is a separate TCP socket that never blocks).
     * The boot log above keeps the default 100 ms so a slow enumerating host
     * still gets the whole banner. */
    Serial.setTxTimeoutMs(0);
}

void appStep(void) {
    /* Anything that blocks here makes the car look frozen while Wi-Fi stays up, so
     * a long gap is reported instead of being felt. Bench routines from the serial
     * console are expected to block; they say so through consoleBusy(). */
    const uint32_t now = millis();
    if (s_lastStep && !consoleBusy() && (now - s_lastStep) > LOOP_STALL_MS)
        say("WARN loop stalled %lu ms", (unsigned long)(now - s_lastStep));
    s_lastStep = now;

    /* Requested from the page (CAL key). Deliberately here and not in the WebSocket
     * handler: this is the one place in the loop where a 1.2 s still window costs
     * nothing, because the motors are already stopped and appCalibrate() declares
     * the gap to the watchdog. */
    if (s_calPending) { s_calPending = false; appCalibrate(); }

    schedRun(kTasks, kTaskCount, now);
    delay(1);                            /* a scheduler yield, not a wait: without it the IDLE task on this core starves */
}