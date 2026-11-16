/* SPDX-License-Identifier: MIT */
/* car_console.cpp - serial keys, bench self test, battery / network diagnosis.
 *
 * Every line printed here states the measured value AND the expected one: a bare
 * number tells nobody whether it is wrong.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "src/app/car_config.h"
#include "src/hw/car_hw.h"
#include "src/app/car_log.h"
#include "src/hw/car_battery.h"
#include "src/app/car_console.h"
#include "src/app/car_app.h"

/* the duty the yaw-rate loop asks for at PIVOT_RATE_DPS, derived from the same
 * constants the loop uses, so the bench and the car cannot disagree */
#define PIVOT_TEST_DUTY (PIVOT_DUTY_BASE + (int)(FF_PIVOT_DUTY * PIVOT_RATE_DPS))

static bool s_busy = false;

bool consoleBusy(void) { return s_busy; }

/* Keep the IMU and the encoders sampling while a bench routine blocks the loop. */
static void benchRun(uint32_t ms) {
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        imu.update();
        encoder.update();
        delay(1);
    }
}

/* Battery / ADC. The NORMAL values are sampled FIRST, because the pull tests
 * below deliberately disturb the pad; measuring after them reported a pulled-down
 * 1.0 V node instead of the real 1.95 V one. */
void consoleBattery(void) {
    const int   adc = batteryAdc();
    const float tap = batteryPinMv();
    const float v   = batteryVolts();
    const int   pct = batteryPercent();
    say("[bat] adc %d counts, tap %.0f mV -> %.2f V (%d%%)", adc, (double)tap,
        (double)v, pct);

    /* Now the pad is allowed to be abused: does anything external sit on it? */
    const int   cFloat = batteryRawCountsPulled(BAT_PULL_FLOAT);
    const int   cUp    = batteryRawCountsPulled(BAT_PULL_UP);
    const int   cDown  = batteryRawCountsPulled(BAT_PULL_DOWN);
    (void)batteryTapMv(BAT_PULL_FLOAT);            /* leaves the pad floating */
    say("[bat] raw counts float %d / up %d / down %d (same pin, three pull states)",
        cFloat, cUp, cDown);
    if (cDown < cFloat - 400 && cUp > cFloat + 400)
        say("[bat] VERDICT both pulls move the pin: it is the divider tap, and %.2f V "
            "is the 2S battery rail held by the BQ25886RGER", (double)v);
    else
        say("[bat] VERDICT the pulls do NOT move the pin: an active source or a very "
            "stiff node drives it, so trust %.0f mV (multimeter-checkable) over the "
            "divider maths", (double)tap);
}

void consoleHelp(void) {
    say("serial keys: c=IMU zero calibration  t=self test  s=stop  h=heading hold  "
        "l=left  r=right  y=yaw/status  i=network/page status  b=battery/ADC  ?=help");
    say("direction is configured by the three macros in car_motor.h "
        "(MOTOR_SWAP_LR / MOTOR_L_INV / MOTOR_R_INV): swapLR=%d M1 inv=%d M2 inv=%d",
        MOTOR_SWAP_LR, MOTOR_L_INV, MOTOR_R_INV);
}

void consoleState(void) {
    /* ycmd / ymeas are the yaw loop itself: commanded vs measured rotation rate.
     * dv is the wheel-speed differential the loop asks for: two motors that do not
     * match need exactly this offset to keep the car straight. */
    say("STATE mode=%-4s trn=%+d duty=%3d/%3d rpm=%+5.0f/%+5.0f "
        "yaw=%+6.1f tyaw=%+6.1f ycmd=%+5.0f ymeas=%+5.0f dv=%+5.1f "
        "travel=%4.0f/%4.0fmm cal=%d",
        ctl.modeName(), ctl.turnDir(), ctl.dutyL(), ctl.dutyR(),
        (double)encoder.rpmL(), (double)encoder.rpmR(),
        (double)imu.yawDeg(), (double)ctl.targetYaw(),
        (double)ctl.yawRateCmd(), (double)ctl.yawRateMeas(),
        (double)ctl.yawDiffRpm(),
        (double)ctl.turnTravelMm(), (double)ctl.turnArcMm(),
        imu.calibrated() ? 1 : 0);
}

/* ── bench self test: drive BOTH channels and report what each sensor saw ──
 * The two sides of this chassis are wired mirrored, so the SAME duty must make
 * both wheels turn the same physical way. That is the only measurement that can
 * prove the pair agrees: one channel alone cannot, because "+duty raises the
 * count" is true per channel by construction and says nothing about the other. */
void consoleSelfTest(void) {
    s_busy = true;
    ctl.stop();
    benchRun(200);

    say("");
    say("=== SELF TEST (car free to spin: lift it or clear 1 m around) ===");
    say("imu cal=%d yaw=%+.1f roll %+.1f pitch %+.1f (mounting tilt)  gyro bias "
        "%.2f/%.2f/%.2f dps",
        imu.calibrated() ? 1 : 0, (double)imu.yawDeg(),
        (double)imu.rollDeg(), (double)imu.pitchDeg(),
        (double)imu.gyroBias(0), (double)imu.gyroBias(1), (double)imu.gyroBias(2));
    /* Raw counts and tap mV first: those are the numbers a meter can be compared
     * against. Volts are only ever tap x 130/30 (the tap is on the battery rail). */
    say("battery adc %d counts  tap %.0f mV  -> %.2f V (%d%%)", batteryAdc(),
        (double)batteryPinMv(), (double)batteryVolts(), batteryPercent());

    /* Both wheels forward, then backward. Duty, encoder counts and rpm are all
     * printed: with only rpm a frozen wheel and a dead encoder look identical. */
    int32_t dLf = 0, dRf = 0;
    float rpmLf = 0.0f, rpmRf = 0.0f;
    for (int step = 0; step < 2; step++) {
        const int duty = step ? -40 : 40;
        const float yaw0 = imu.yawDeg();
        const int32_t l0 = encoder.left(), r0 = encoder.right();
        hwSetWheels(duty, duty);
        const int al = motor.left(), ar = motor.right();   /* what was really applied */
        benchRun(900);
        const float rpmL = encoder.rpmL(), rpmR = encoder.rpmR();
        const int32_t dL = encoder.left() - l0, dR = encoder.right() - r0;
        benchRun(300);
        hwSetWheels(0, 0);
        benchRun(400);
        say("both %+3d duty (raw M1/M2 %+3d/%+3d) -> rpm L %+7.1f R %+7.1f  "
            "counts M1 %+6d M2 %+6d  yaw %+.1f",
            duty, al, ar, (double)rpmL, (double)rpmR, (int)dL, (int)dR,
            (double)(imu.yawDeg() - yaw0));
        if (step == 0) { dLf = dL; dRf = dR; rpmLf = rpmL; rpmRf = rpmR; }
    }

    /* Single-wheel pivot, driven through the SAME left/right mapping the arrow keys
     * use (going straight to a motor channel here is how a bench test ends up
     * disagreeing with real behaviour). Left turn = right wheel forward. */
    const bool m1IsRight = MOTOR_SWAP_LR != 0;
    const int pL = m1IsRight ? PIVOT_TEST_DUTY : 0;
    const int pR = m1IsRight ? 0 : PIVOT_TEST_DUTY;
    const float yawA = imu.yawDeg();
    const int32_t lA = encoder.left(), rA = encoder.right();
    hwSetWheels(pL, pR);
    benchRun(1500);
    hwSetWheels(0, 0);
    benchRun(300);
    const float dYaw = imu.yawDeg() - yawA;
    const int32_t dLt = encoder.left() - lA, dRt = encoder.right() - rA;
    say("left turn: M1 %+d M2 %+d (M1 is the %s wheel) -> yaw %+.1f deg "
        "(expect POSITIVE, |d| > 20)  counts M1 %+d M2 %+d",
        pL, pR, m1IsRight ? "right" : "left", (double)dYaw, (int)dLt, (int)dRt);

    /* Verdicts: each one names the next action instead of a bare number. */
    const bool wheelsMoved = (labs((long)dLf) + labs((long)dRf)) >= 20;
    if (!wheelsMoved) {
        say("VERDICT NO wheel movement at all -> motor power, battery switch, motor "
            "connector, or the wheels are blocked");
        say("        (nothing else can be judged until the wheels turn: fix this first)");
    } else {
        if (fabsf(rpmLf) < 20.0f || fabsf(rpmRf) < 20.0f)
            say("VERDICT counts move but rpm reads 0 -> encoder window/sign problem");
        if ((dLf > 0) != (dRf > 0))
            say("VERDICT the wheels turn in OPPOSITE directions -> set MOTOR_L_INV or "
                "MOTOR_R_INV to %d - flip the channel that ran backward",
                !MOTOR_L_INV);
        else
            say("VERDICT both wheels drive the same way (%s)", dLf > 0 ? "forward" : "reverse");
        if (fabsf(dYaw) < 10.0f)
            say("VERDICT the car did not rotate but the wheels did -> check that only one "
                "wheel drives (outer) and the inner one is free to coast");
        else if (dYaw < 0.0f)
            say("VERDICT turn direction is INVERTED -> set MOTOR_SWAP_LR to %d (it is %d)",
                !MOTOR_SWAP_LR, MOTOR_SWAP_LR);
        else
            say("VERDICT turn direction is CORRECT (MOTOR_SWAP_LR %d)", MOTOR_SWAP_LR);
    }
    say("pivot duty=%d (yaw-rate loop target %.0f deg/s), straight driving moves the "
        "wheel speed targets by +-%.0f rpm", PIVOT_TEST_DUTY, (double)PIVOT_RATE_DPS,
        (double)YAW_DIFF_MAX);
    say("dir: swapLR=%d M1 inv=%d M2 inv=%d  enc signs %+d/%+d", MOTOR_SWAP_LR,
        MOTOR_L_INV, MOTOR_R_INV, ENC_L_SIGN, ENC_R_SIGN);
    say("=== SELF TEST DONE ===");
    say("");
    s_busy = false;
}

void consolePoll(void) {
    while (Serial.available() > 0) {
        const int c = Serial.read();
        switch (c) {
            case 't': consoleSelfTest(); break;

            case 'c':
                /* Zero-rate calibration: gyro bias + attitude seed from gravity.
                 * The car only has to be STILL; its mounting angle is free.
                 * Same function the page's CALIBRATE key uses. */
                appCalibrate();
                break;

            case 's': ctl.stop(); say("[cmd] STOP"); break;

            case 'l': ctl.pivot(1);  say("[cmd] PIVOT left (right wheel only)"); break;
            case 'r': ctl.pivot(-1); say("[cmd] PIVOT right (left wheel only)"); break;

            case 'h':
                ctl.holdHeading(!ctl.headingHold());
                say("[cmd] heading hold %s", ctl.headingHold() ? "ON" : "OFF");
                break;

            case 'y':
                /* roll/pitch here are the MOUNTING tilt: the board does not have to
                 * sit flat, yaw is measured about the true vertical either way */
                say("[yaw] %+.1f deg (raw %+.1f)  rate %+.1f dps  roll %+.1f pitch %+.1f  "
                    "cal=%d still=%d dev=%.2f dps  hdg=%s",
                    (double)imu.yawDeg(), (double)imu.yawRawDeg(),
                    (double)imu.yawRateDps(),
                    (double)imu.rollDeg(), (double)imu.pitchDeg(),
                    imu.calibrated() ? 1 : 0, imu.still() ? 1 : 0,
                    (double)imu.stillDevDps(), ctl.headingHold() ? "ON" : "OFF");
                break;

            case 'i': {
                /* Everything needed to answer "why can I not open the page": the
                 * link, the address, whether requests arrive, and how far they get. */
                const int32_t age = web.lastCmdAgeMs();
                say("[net] wifi=%s ip=%s host=http://%s.local port=%u rssi=%d dBm",
                    web.wifiUp() ? "up" : "DOWN", WiFi.localIP().toString().c_str(),
                    HOSTNAME, (unsigned)HTTP_PORT, (int)WiFi.RSSI());
                say("[net] requests=%lu pages=%lu failed=%lu parked=%d pageOut=%d ws=%s "
                    "cmdAge=%ld ms",
                    (unsigned long)web.requests(), (unsigned long)web.pagesServed(),
                    (unsigned long)web.pagesFailed(),
                    web.parked(), web.pageOuts(), web.connected() ? "OPEN" : "closed",
                    (long)age);
                if (!web.wifiUp())
                    say("[net] Wi-Fi is down -> check SSID/password in car_config.h, or the AP");
                else if (web.requests() == 0)
                    say("[net] nobody has reached the board -> wrong IP/hostname, or the "
                        "phone is on another network");
                else if (web.pagesFailed())
                    say("[net] the page never left the board (%lu failures) -> look for "
                        "[web] page FAILED lines on the console", (unsigned long)web.pagesFailed());
                else if (web.pagesServed() == 0)
                    say("[net] requests arrive but no page completed -> the response is "
                        "being cut; look at the pageOut count above");
                else if (!web.connected())
                    say("[net] page is served but the WebSocket never opened -> reload the "
                        "page; if it keeps failing the <script> did not run");
                else if (age < 0 || age > 5000)
                    say("[net] connected but no command for %ld ms -> the page is open and "
                        "idle, or it is not sending", (long)age);
                else
                    say("[net] chain looks healthy: page served, WebSocket open, commands "
                        "arriving");
                break;
            }

            case 'b': consoleBattery(); break;
            case '?': consoleHelp(); break;

            default: break;
        }
    }
}