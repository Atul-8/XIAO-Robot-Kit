/* SPDX-License-Identifier: MIT */
/* car_battery.h - battery sense on D16: raw ADC, tap mV, pack volts, rough SOC. */
#ifndef CAR_BATTERY_H
#define CAR_BATTERY_H

#include <Arduino.h>

/* Pad pull states for batteryTapMv(): the tap is 23 kOhm (100K || 30K), which is
 * high enough that the chip's own pull resistors move the reading. Reading with
 * all three states is what tells a real divider apart from a pulled-up pin. */
enum { BAT_PULL_FLOAT = 0, BAT_PULL_UP, BAT_PULL_DOWN };

/* Must be called once before any read: resolution, attenuation, no pad pull.
 * The ADC conversion itself is analogReadMilliVolts(), i.e. the chip's own
 * reference and its per-chip eFuse calibration curve - not a raw-count formula. */
void batteryBegin(void);

/* Calibrated ADC millivolts AT THE DIVIDER TAP, i.e. what a multimeter would read
 * on that pin - not the battery voltage. Averaged, because the tap is 23 kOhm
 * (100K || 30K) and that is a high source impedance for the S3 ADC. */
float batteryPinMv(void);

/* Same, but with the pad pull forced to `mode` (BAT_PULL_*). Used by the `b`
 * diagnostic only; leaves the pad floating again. */
float batteryTapMv(int mode);

/* Raw 12-bit conversion with the pad pull forced, after letting the node settle.
 * Diagnostic only: it is what proves the pin is a divider tap and not a stiff
 * source, and the counts are comparable with a scope/meter claim. */
int batteryRawCountsPulled(int mode);

/* Raw ADC value, averaged over 4 conversions: the number the page shows next to
 * the volts, so the ADC can always be read without doing maths. */
int batteryAdc(void);

/* Pack voltage = tap * (H + L) / L, in volts. The tap sees the battery connector,
 * which the BQ25886RGER holds near 8.3-8.4 V while the board is powered from
 * Type-C - a 2S rail, not the 5 V input (see car_config.h). */
float batteryVolts(void);

/* Rough state of charge, 0..100, clamped, over 8.40 V (full) to 6.00 V (empty).
 * Linear on purpose: this is a teaching display, not a fuel gauge. */
int batteryPercent(void);

#endif /* CAR_BATTERY_H */