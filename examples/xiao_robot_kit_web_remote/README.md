# xiao_robot_kit_web_remote — drive the car from a phone browser

The board hosts its own HTTP page and WebSocket server on one TCP port, so a phone or desktop
browser on the same Wi-Fi drives the car at `http://xiaorobot.local`. No app, no third-party
library — only the ESP32 Arduino core (WiFi + Wire); the HTTP handling, the WebSocket handshake and
the page itself are hand-written.

Each wheel has an encoder speed loop; a gyro heading loop sits on top of it, so the car keeps a
straight line even though the two geared motors are not equal.

## What the page does

| Control | Behavior |
| --- | --- |
| ▲ / ▼ | Press and hold: 150 mm/s forward or reverse; release coasts |
| ◀ / ▶ | Press and hold: single-wheel pivot — the inner wheel stops, the outer wheel drives |
| ▲ + ◀ | Held together: the car arcs while it drives |
| Red center key | E-stop; Space / Esc work too |
| SET FORWARD | Current heading becomes yaw 0 |
| HOLD | Keeps repeating the last direction after the key is released |
| CALIBRATE | Zero the gyro (car must be still, 1.2 s) — the same action as serial `c` |
| Battery badge | `ADC 2312 · 8.47 V`, raw ADC counts plus volts from the divider tap |
| Log window | Every line the firmware prints on serial is mirrored here |

Every key tracks its own pointer, so ▲ + ◀ really are held at the same time, and releasing one
never cancels the other. If the WebSocket is down the page says `NOT CONNECTED to the car` on the
next press instead of pretending it worked.

## How it drives

- Per wheel: feed-forward + PI on encoder rpm (50 ms window, 10 ms control period).
- Heading hold: heading error → differential rpm → **added to the speed targets** → per-wheel PI →
  duty, damped by the measured gyro rate. It is always on; serial `h` toggles it for comparison.
- Pivot and arcs use a **gyro rate loop** (`60` / `45` deg/s), so one key press turns the same
  amount on wood, carpet or a half-empty battery.
- The page re-sends the current command every 300 ms; with no command for 1.5 s the firmware stops
  the car, so a Wi-Fi drop coasts to a stop by itself.

## Pins

| Purpose | Pins |
| --- | --- |
| Motor M1 IN1 / IN2 | D1 (IO2) / D2 (IO3) |
| Motor M2 IN1 / IN2 | D19 (IO11) / D18 (IO12) |
| Encoder M1 A / B | D3 (IO4) / D14 (IO41) |
| Encoder M2 A / B | D17 (IO13) / D10 (IO9) |
| IMU LSM6DS3TR-C | I2C SDA D4 (IO5) / SCL D5 (IO6), address `0x6A` |
| Battery divider | D16 (IO10), 100K : 30K on the battery connector |

The divider sits on the battery connector of the BQ25886RGER PMIC, i.e. the 2S pack rail
(~8.3–8.4 V), not the 5 V Type-C input. Thresholds are 2S: 8.40 V full, 6.00 V empty.

## Direction — three macros, nothing automatic

`src/hw/car_motor.h` decides the wiring. Nothing is probed, learned or corrected at runtime, so the
car behaves the same after every flash.

```cpp
MOTOR_SWAP_LR   /* 1 = M1 is physically the RIGHT wheel (duty and encoder map together) */
MOTOR_L_INV     /* 1 = flip M1 */
MOTOR_R_INV     /* 1 = flip M2 */
```

| Symptom | Change |
| --- | --- |
| Forward drives the car backwards | flip **both** `MOTOR_L_INV` and `MOTOR_R_INV` |
| Only one wheel goes the wrong way | flip that channel's `MOTOR_x_INV` |
| Left/right respond mirrored, or the car drifts while going straight | flip `MOTOR_SWAP_LR` |

`ENC_L_SIGN` / `ENC_R_SIGN` (`src/hw/car_encoder.h`) are **not** knobs: they define what "forward"
means for the speed loop. Boot prints the active combination as
`[mot] direction: swapLR=1 M1 inv=0 M2 inv=1`.

## Build and run

1. Set the Wi-Fi credentials in `src/app/car_config.h` (or pass `-DWIFI_SSID=... -DWIFI_PASS=...`):

   ```cpp
   #define WIFI_SSID  "HelloWorld"
   #define WIFI_PASS  "HelloWorld"
   ```

2. Compile and upload (ESP32 core 3.x):

   ```bash
   arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3_Plus examples/xiao_robot_kit_web_remote
   arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32S3_Plus -p COMx examples/xiao_robot_kit_web_remote
   ```

3. Open the serial monitor at 115200 — it prints the IP — and browse to `http://xiaorobot.local` or
   that IP. Let the car stand still for ~2 s after power-on: it calibrates and never moves the
   wheels during boot.

## Serial keys (no page needed)

| Key | Action |
| --- | --- |
| `c` | Gyro zero calibration — car still, any mounting attitude |
| `t` | Self-test: both wheels forward / reverse / left turn, measured vs expected per line, then a verdict |
| `s` | Stop |
| `l` / `r` | Continuous pivot left / right until `s` |
| `h` | Heading hold on / off |
| `y` | Yaw, calibration state, still deviation |
| `i` | Network and page state (requests, pages, WebSocket, command age) |
| `b` | Battery / ADC self-check: counts, tap mV, volts, percent, verdict |
| `?` | Help |

`t` prints what each sensor measured, so a dead wheel, an uncounted encoder and a wrong motor sign
look different: counts not moving = wheel never turned; counts moving but rpm 0 = encoder problem;
the two wheels' counts with opposite signs = flip that motor's `MOTOR_x_INV`.

## Protocol (WebSocket text frames)

```
page → board   DRV:<mm/s>,<turn>   the only command the page sends; turn +1 = left, -1 = right
               SPD:<mm/s>          line speed only, 0 = coast
               PIVOT:<1|0|-1>      continuous pivot (serial clients, tests)
               TURN:<deg>          pivot by angle, + = counter-clockwise
               HDG:<1|0>           heading hold on / off
               FWD                 current heading := yaw 0
               CAL                 run the gyro zero calibration
               STOP                e-stop

board → page   T:<mm>,<adc>,<tapMv>,<rpmL>,<rpmR>,<cal>    5 Hz
               L:<text>            one serial line, mirrored to the log window
               OK … / ERR
```

## Files

```
xiao_robot_kit_web_remote.ino    setup() and loop(), nothing else
src/app/    car_app (boot, command routing, telemetry, task table), car_config (all knobs),
            car_sched (cooperative task table), car_log (serial + page log), car_console (keys),
            car_tick (hardware timer heartbeat)
src/ctl/    car_control — cascade control and motion state machine
src/hw/     car_hw (pins + glue), car_motor, car_encoder, car_imu (Mahony Euler), car_battery
src/net/    car_web — HTTP page + WebSocket, non-blocking
```

Arduino's `<sketch>/src/` layout: files are compiled recursively and headers are included from the
sketch root (`#include "src/hw/car_motor.h"`).

No job waits for anything — the control loop, the page parser and the WebSocket writer all run
from one task table, so a slow browser, a sleeping phone or a closed serial monitor cannot stall the
motors. The page is ~14 KB and is pushed to the socket in MSS-sized steps, again without blocking.

## Limits and things to measure

- **No magnetometer**: yaw is relative and drifts slowly; heading hold keeps a line straight, it
  does not know north.
- Wheel diameter (`WHEEL_DIA_MM 50`) and track (`TURN_TRACK_MM 100`) are kit values — measure yours
  and update `src/ctl/car_control.h`; odometry and angle turns scale with them.
- Mounting attitude is free: calibration measures gravity, yaw is taken about the true vertical
  axis, so a tilted or upside-down board needs no code change — only "be still" while calibrating.
- Gains were tuned in host simulation against a friction/dead-zone model. If you change motors or
  battery, tune inner loop first, outer loop last (see the comments in `car_control.h`).
- The differential never reverses a wheel, so heading correction fades below ~40 mm/s by design.