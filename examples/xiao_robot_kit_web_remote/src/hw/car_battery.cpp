/* SPDX-License-Identifier: MIT */
/* car_battery.cpp - battery sense on D16: raw ADC, calibrated tap mV, pack volts.
 *
 * The tap sits on the BATTERY connector, which the BQ25886RGER PMIC holds around
 * 8.3-8.4 V whenever the board is powered (2S rail). Every layer is exposed so it
 * can be checked with a meter: counts, tap mV, then volts.
 */
#include <Arduino.h>
#include <math.h>          /* fabsf: the host stub does not pull in math.h */
#include <driver/gpio.h>
#if defined(SOC_RTCIO_PIN_COUNT) && (SOC_RTCIO_PIN_COUNT > 0)
#include <driver/rtc_io.h>
#include <soc/soc_caps.h>
#endif
#include "src/app/car_config.h"
#include "src/hw/car_battery.h"

/* The pad has TWO pull facilities and they are separate registers: the digital
 * ones (gpio_set_pull_mode) and the RTC/analog ones (rtc_gpio_*). A pin used as
 * an analog input is muxed to the RTC domain, so disabling only the digital pull
 * can leave an ~45 kOhm pull-up LIVE. Round 9 measured "float == up == down",
 * i.e. the digital call alone did nothing. Both are set here. */

static void padPull(int mode) {
    const gpio_num_t g = (gpio_num_t)BAT_PIN;
#if defined(SOC_RTCIO_PIN_COUNT) && (SOC_RTCIO_PIN_COUNT > 0)
    if (rtc_gpio_is_valid_gpio(g)) {
        (void)rtc_gpio_init(g);              /* route the pad to the RTC/analog mux */
        const bool enUp   = (mode == BAT_PULL_UP);
        const bool enDown = (mode == BAT_PULL_DOWN);
        (void)(enUp   ? rtc_gpio_pullup_en(g)   : rtc_gpio_pullup_dis(g));
        (void)(enDown ? rtc_gpio_pulldown_en(g) : rtc_gpio_pulldown_dis(g));
    }
#endif
    if (mode == BAT_PULL_UP)        gpio_set_pull_mode(g, GPIO_PULLUP_ONLY);
    else if (mode == BAT_PULL_DOWN) gpio_set_pull_mode(g, GPIO_PULLDOWN_ONLY);
    else                            gpio_set_pull_mode(g, GPIO_FLOATING);
}

void batteryBegin(void) {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);   /* ~3.1 V full scale: the tap sits near 1.2 V */
    /* THIS is the line that matters on this board: the divider is 100K on top of
     * 30K, so the tap is a weak node. Any pull resistor left enabled on the pad
     * (internal ~45 kOhm to 3.3 V) sits in PARALLEL with the divider and lifts the
     * tap from 1.15 V to ~1.95 V, which the maths then reports as ~8.4 V on a 5 V
     * supply. 5 V * 30/130 = 1.154 V is the whole truth. */
    padPull(BAT_PULL_FLOAT);
}

float batteryPinMv(void) {
    /* Pin state first, EVERY time. Measured on this board: plain analogRead() of a
     * pad whose internal pull-down is still enabled reports ~1.0 V (1084 counts),
     * while analogReadMilliVolts() - which re-inits the pad itself - reports the
     * real 1.95 V node (2151 counts). Two answers for one pin means the state, not
     * the ADC, was the variable. So the state is pinned down here, once. */
    padPull(BAT_PULL_FLOAT);
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) sum += analogReadMilliVolts(BAT_PIN);
    return (float)sum / 4.0f;
}

/* Plain reads: the CALLER owns the pin state. batteryTapMv() sets the pull it
 * wants and then samples - resetting the pad here would delete the test. */
static float readAvg(int n) {
    uint32_t sum = 0;
    for (int i = 0; i < n; i++) sum += analogReadMilliVolts(BAT_PIN);
    return (float)sum / (float)n;
}

/* Settle, do not sleep: after switching a pull the node needs a moment (23 kOhm
 * source impedance, plus whatever capacitor the board has). Sampling instantly
 * reports the OLD value, which is exactly how "pull-down gives 1970 mV" could
 * mean nothing at all. Poll until two readings agree. */
float batteryTapMv(int mode) {
    padPull(mode);
    float mv = readAvg(4);
    for (int t = 0; t < 24; t++) {        /* at most 24 * 5 ms = 120 ms */
        delay(5);                         /* diagnostic path only, never the loop */
        const float now = readAvg(4);
        const bool settled = fabsf(now - mv) < 8.0f;
        mv = now;
        if (settled) break;
    }
    padPull(BAT_PULL_FLOAT);
    return mv;
}

int batteryRawCountsPulled(int mode) {
    padPull(mode);
    delay(40);                            /* same settle reason as batteryTapMv */
    const int c = analogRead(BAT_PIN);
    padPull(BAT_PULL_FLOAT);
    return c;
}

float batteryVolts(void) {
    /* VBAT = tap * (H + L) / L. See car_config.h: the tap is on the battery
     * connector, which the PMIC holds at ~8.4 V while running from Type-C. */
    return batteryPinMv() * (BAT_HIGH_K + BAT_LOW_K) / BAT_LOW_K / 1000.0f;
}

int batteryPercent(void) {
    const float span = BAT_FULL_V - BAT_EMPTY_V;
    const int pct = (int)((batteryVolts() - BAT_EMPTY_V) * 100.0f / span + 0.5f);
    return pct < 0 ? 0 : (pct > 100 ? 100 : pct);
}

int batteryAdc(void) {
    padPull(BAT_PULL_FLOAT);              /* raw counts are only meaningful on a
                                           * pad that is known to be floating */
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) sum += (uint32_t)analogRead(BAT_PIN);
    return (int)(sum / 4u);
}
