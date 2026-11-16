# xiao_robot_kit_ble_control — BLE command-control chassis firmware

ESP-IDF firmware for the XIAO Robot Kit. Two entry points expose **one command set**: a BLE GATT
characteristic and the native USB Serial/JTAG shell. Write `drive 20 20` from a phone app or from a
terminal and the same dispatcher runs it.

The BLE device name is not compiled in — it is stored in NVS and set with `btname set <name>`, so
one firmware image can be flashed to a whole batch of boards and each unit named afterwards. An
unnamed board advertises as `XIAO Robot Kit`.

## Hardware

| Item | Detail |
| --- | --- |
| Board | XIAO ESP32-S3 Plus, 16 MB flash, 8 MB octal PSRAM |
| Motor driver | GC9112 (U14 / U16), 20 kHz PWM |
| Motors | 2× geared DC motor with quadrature encoder |
| Servos | 2× arm servo, 50 Hz PWM, 620–2500 µs |
| IMU | LSM6DS3TR-C on I2C `0x6A` (±500 dps, gravity axis from a still-car calibration) |
| RGB strip | 10× WS2812 on D13 / GPIO40, GRB order, RMT driven |
| Console | USB Serial/JTAG (not UART0 — UART0 is the telemetry port) |

| Motor | Body position | IN1 | IN2 |
| --- | --- | --- | --- |
| M1 / J8 | Right wheel | D1 / GPIO2 | D2 / GPIO3 |
| M2 / J9 | Left wheel | D19 / GPIO11 | D18 / GPIO12 |

| Servo | Body position | Connector | GPIO |
| --- | --- | --- | --- |
| SERVO1 | Right arm | J1 | D15 / GPIO42 |
| SERVO2 | Left arm | J5 | D9 / GPIO8 |

M2 is mounted mirrored; its polarity and encoder direction are calibrated in the firmware, so
`forward` turns both wheels forward and both counters count up.

## Build and flash

```bash
source <ESP_IDF_ROOT>/export.sh          # ESP-IDF v6.1
cd examples/xiao_robot_kit_ble_control
idf.py -p <serial-port> flash monitor
```

Target, flash size, PSRAM mode, partition table and console are pinned in `sdkconfig.defaults` and
`partitions.csv`, so a fresh clone rebuilds the same image. On Windows `run.bat` builds only;
`run.bat COMx` builds, flashes and opens the monitor (it expects `IDF_PATH` and
`IDF_PYTHON_ENV_PATH`, with a default under `%USERPROFILE%\esp`).

## Console

`monitor` gives a `robot> ` prompt and `help` lists every command. Everything below works
identically over BLE, because BLE text is handed to the same parser.

### Movement (open loop, runs until `stop`)

| Command | Effect |
| --- | --- |
| `motor <1-2> <-100..100>` | One motor, sign = direction |
| `motorinv <1-2> <0\|1>` | Flip one motor's polarity |
| `drive <left> <right>` | Both sides, `-100..100` |
| `forward [1..100]` / `backward [1..100]` | Straight drive |
| `left` / `right` | In-place turn, one side drives |
| `stop` | Stop motors and cancel any running preset |

### Speed loop

| Command | Effect |
| --- | --- |
| `speed <left> <right>` | Closed-loop wheel speed, `-100..100` → target count/s (100 = 5766 CPS) |
| `enc` / `encreset` | Show / zero the encoder counts |
| `cps` | Measure both wheels over a real 1 s window |
| `cpslog <start\|stop\|list\|summary\|dump ID\|delete ID\|clear\|status>` | Record 40 Hz CPS/PWM samples, persist to SPIFFS |
| `imulog <list\|dump ID\|delete ID\|clear\|status>` | Record IMU samples |
| `encdbg` | Live encoder pin levels |

The loop runs every 50 ms as per-wheel feed-forward + PID, with the feed-forward split by direction
from ground-loaded `cpslog` sweeps. Below `speed 25` static friction wins and the wheel does not move.

### Chassis presets (they all go through the speed loop, not raw PWM)

| Command | Effect |
| --- | --- |
| `move_forward` / `move_backward` | IMU heading hold, both wheels ±55, 1 s |
| `turn_left` / `turn_right` | one wheel 70, other 0, 1.5 s |
| `spin_clockwise` / `spin_counterclockwise` | ±100 / ∓100 until the IMU has turned 360° |
| `reverse_park` | reverse arc, left −50 / right −100, until the IMU reads 90°, 10 s timeout |
| `greeting` | right-arm wave while driving +55 for 1.5 s, then turn back to the start heading |
| `emotion_happy` | arm raise + 360° spin |
| `emotion_scared` | arms close while reversing at −80 for 500 ms |
| `emotion_angry` | arms open/close twice while wobbling ±100 for 2 × 2 × 200 ms |
| `dance_take_me_hand` / `dance_believer` / `dance_new_believer` | arm + chassis choreography |

IMU presets need a valid still-car calibration: the firmware calibrates automatically ~1.5 s after
power-on with the motors held low, and `imu calibrate` re-runs it on demand. Without it those
commands refuse to start.

### Arms, tuning, housekeeping

| Command | Effect |
| --- | --- |
| `servo <1-2> <0..180>` | Right = 1 (J1), left = 2 (J5); 90 is the installed mechanical centre |
| `servooff <1-2>` | Release that servo's PWM |
| `handshake` / `wave_hand` / `open_arms` / `raise_both_arms` / `close_arms` | Arm presets; `armstop` cancels |
| `tune speed <action> <left> <right>` | Retune a preset in RAM; `tune save reverse_park` persists it to NVS |
| `tune arm <pose> <servo1> <servo2>` / `tune show` / `tune reset` | Arm pose tuning |
| `imu <status\|sample [n]\|reg <addr>\|reinit\|calibrate>` | IMU diagnostics |
| `battery` | Battery voltage on J2 |
| `rgb <r> <g> <b>` / `rainbow` / `rgboff` | RGB strip |
| `btname set <name>` / `show` / `reset` | Persistent BLE name (1–26 chars, `A-Z a-z 0-9 . _ -`) |
| `status` | Version and all peripheral state |

## BLE interface

| Item | Value |
| --- | --- |
| Service | `a2f39c10-7e4b-4d21-8a61-0f3bc9d15001` |
| Command / status characteristic | `a2f39c11-7e4b-4d21-8a61-0f3bc9d15001` |
| Properties | Read, Notify, Write, Write Without Response |
| Write | Any shell command as UTF-8 text, up to 63 bytes |
| Reply | `OK\|<command>\|$`, or `ERR\|UNKNOWN\|$` / `ERR\|EXEC\|$` / `ERR\|LENGTH\|$` |

Write Without Response and subscribe to notifications, e.g. with nRF Connect:

```text
drive 20 20      forward
drive -20 20     in-place left
stop
```

A disconnect, a NimBLE reset or any manual motor command stops the motors and cancels a running
preset. The firmware re-arms advertising after a disconnect, so the board stays scannable; a phone
that already cached the old name has to forget the device once.

## VOFA+ telemetry on J10

J10 is UART0 at 115200-8N1, 3.3 V: pin 1 TX = D6 / GPIO43, pin 2 RX = D7 / GPIO44, pin 4 GND.
Every 100 ms it prints one FireWater line, ten columns in this order:

```text
60,60,3460,3460,1234,1250,3010,2995,58,60
```

`set_l, set_r, pid_l, pid_r, enc_l, enc_r, cps_l, cps_r, pwm_l, pwm_r` — the `-100..100` request,
the internal CPS target per wheel, cumulative encoder counts, counts/s and PID output. Encoder and
CPS columns keep streaming while the loop is idle.

## Files

```text
main/main.c              entry point, USB shell, command table
main/motor_control.*     GC9112 PWM and direction
main/encoder.*           dual quadrature counting and polarity calibration
main/speed_control.*     feed-forward + PID speed loop, CPS logging hooks
main/chassis_action.*    IMU-guided presets and still-car calibration, NVS tuning
main/arm_action.*        arm presets, pose interpolation, runtime pose tuning
main/dance.*             choreography engine
main/servo.*             LEDC servo PWM with centre calibration
main/imu_lsm6ds3tr.*     IMU driver;  main/imu_heater.*  heater;  main/imu_spin_log.*  spin logs
main/battery.*           battery ADC
main/cps_log.*           RAM ring + SPIFFS persistence for CPS samples
main/rgb_led.*           WS2812 strip over RMT
main/ble_motor.*         NimBLE GATT service, command dispatch, NVS device name
main/vofa_telemetry.*    J10 FireWater frames
sdkconfig.defaults / partitions.csv    board configuration and flash layout
```

## Before you drive

- Keep the wheels off the ground for the first test and be able to cut motor power.
- The motors run from the kit's motor supply, never from the XIAO 3.3 V pin; XIAO, GC9112 and the
  motor supply share a ground.
- Open-loop commands (`drive`, `forward`, `motor` …) keep running until `stop`.
- A servo driven outside its installed range can stall against the arm's mechanical stop — move in
  small steps from `servo <id> 90`.