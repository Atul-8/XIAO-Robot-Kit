/* SPDX-License-Identifier: MIT */
/* car_config.h - every knob of the sketch in one place.
 *
 * Nothing else in this sketch should contain a magic number: if a value has to
 * be tuned on the bench, it belongs here, with the unit in the name.
 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* -- Wi-Fi / HTTP (override with -DWIFI_SSID=... at build time) -- */
#ifndef WIFI_SSID
#define WIFI_SSID  "HelloWorld"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS  "HelloWorld"
#endif
#define HOSTNAME     "xiaorobot"
#define HTTP_PORT    80

/* -- task periods (cooperative scheduler, see car_sched.h) -- */
#define TELEMETRY_MS   200      /* T: frame to the page, 5 Hz */
#define STATE_MS       500      /* STATE line while the car is not idle */
#define CONSOLE_MS     20       /* serial key scan */
#define LOOP_STALL_MS  300      /* a gap longer than this is reported */

/* -- bench / boot -- */
#define CALIB_MS       1200     /* still window for the gyro zero-rate calibration */

/* -- battery --
 * Sense path: D16 (GPIO10, ADC1_CH9) on the tap of a 100K:30K divider across the
 * BATTERY connector. The 5 V Type-C is upstream of a BQ25886RGER PMIC, which
 * charges/boosts the pack, so that connector sits around 8.3-8.4 V whenever the
 * board is powered - it is a 2S Li-ion rail, NOT the 5 V input. That is why an
 * 8.5 V reading on a 5 V supply is correct: 1.94 V on the tap x 130/30 = 8.41 V,
 * and a multimeter on the tap agrees with the ADC to 22 mV.
 *
 * So the ratio below is the schematic's 100K:30K (x4.333) and the thresholds are
 * 2S. The raw ADC value and the tap millivolts are still printed/reported
 * everywhere, because they are the only numbers a meter can be compared against. */
#define BAT_PIN        10        /* D16 = GPIO10, ADC1_CH9 */
#define BAT_HIGH_K     100.0f    /* upper leg, kOhm */
#define BAT_LOW_K      30.0f     /* lower leg, kOhm */
#define BAT_FULL_V     8.40f     /* 2S Li-ion, fully charged */
#define BAT_EMPTY_V    6.00f     /* 2S, stop driving below this */

#endif /* CAR_CONFIG_H */