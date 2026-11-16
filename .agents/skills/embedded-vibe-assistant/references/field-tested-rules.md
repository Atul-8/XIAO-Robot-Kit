# Field-Tested Rules Catalog

Deeper reference behind `SKILL.md`. Each rule is a distilled, **reproducible** lesson
from real ESP32 / Arduino / ESP-IDF debugging sessions. They are written as action
sentences ("when you meet X, do Y") so they can be applied on any machine, in any
repo, without private paths or accounts.

## How to read an entry

- **Rule name** — the trigger scenario.
- **Symptom** — what you observe.
- **Action** — what to do, in order.
- **Anti-pattern** — what NOT to do.

---

## USB / Serial / Console

### R1. USB-CDC `Serial` stalls the loop when no monitor is open
- **Symptom:** wireless/TCP firmware freezes while USB is plugged in but no serial
  monitor is attached; it "unsticks" the moment a monitor opens.
- **Action:** add a build-time switch to disable loop-level `Serial.print` for release;
  or always run with a monitor open during development. Note `if (Serial)` evaluation
  itself can block — safe only when USB is actually unplugged.
- **Anti-pattern:** assuming an ESP32-S3 USB-CDC sketch "runs fine without a serial
  consumer."

### R2. Stuck after flash is usually a boot-loop, not a USB/driver problem
- **Symptom:** flash verifies green but the device sits on the ROM console / download
  port and never runs the app.
- **Action:** grab the boot log **first**. Look for an abort ~tens of ms after reset —
  frequently a duplicate init of an auto-started IDF service returning
  `ESP_ERR_INVALID_STATE`. Degrade gracefully (warn + reuse) instead of aborting.
- **Anti-pattern:** ten rounds of USB/driver/hardware suspicion before reading one
  screen of serial log.

### R3. USB-Serial/JTAG "hard reset" does not reset the chip
- **Symptom:** after flashing over USJ, `Hard resetting via RTS...` prints but the
  device never boots the app.
- **Action:** use `--after watchdog-reset` (hardware-level reset). Verify reset by the
  boot banner / a new COM port — never trust console text alone.
- **Anti-pattern:** relying on `run` or soft-reset commands that a stub flasher silently
  swallows.

### R4. A serial port is an exclusive shared resource
- **Action:** before any diagnostic that opens a port/probe, assume it blocks the user;
  prefer logs / files / offline artifacts. A `timeout`/`kill` on the parent process does
  not release the port — confirm the child is gone.
- **Anti-pattern:** blaming the user's tools when the port was actually held by your own
  leftover monitor process.

---

## Build / Flashing

### R5. Build-cache reuse hides your change
- **Symptom:** `arduino-cli upload` reports "Hash verified" but the board still runs
  old code; the binary size is unchanged after an edit.
- **Action:** `arduino-cli compile --clean --fqbn <fqbn> <sketch>` before `upload`. If
  `--clean` drops `partitions.bin`, run one plain `compile` to regenerate it.
- **Anti-pattern:** trusting the uploader's "Hash verified" text as proof the new
  firmware shipped.

### R6. Flash/upload must name an explicit artifact
- **Action:** every "push bytes to device" command must name the exact build directory /
  file; never rely on a tool's default `build/`/`out/` (it may hold a stale binary for a
  different target). Match product-side metadata (board/target/version) against the
  device's reported identity before flashing.
- **Anti-pattern:** using `--force` / `--ignore-*` to bypass a mismatch — that mismatch
  usually points at a real problem and overrides can brick the device.

### R7. Retry a failed build only after clearing the half-built tree
- **Symptom:** `[N/M] FAILED` with "can't find" a header the build itself generated, or
  `Device or resource busy`.
- **Action:** stop lingering `ccache`/`ninja`/`cmake` processes, delete the half-built
  directory, then reconfigure and rebuild. In incremental ESP-IDF builds, a stalled
  step or `ldgen`/`pyparsing` parse error is often a transient — re-run once before
  touching source.
- **Anti-pattern:** editing code to fix a transient/tooling artifact.

### R8. Cap parallel compiles on constrained hosts
- **Action:** run at most ~2 concurrent `idf.py build` jobs, wait between batches, fall
  back to serial. Heavy parallel builds trigger SIGTERM / exit-143 resource races.

---

## Networking (WiFi / BLE / TCP)

### R9. `WiFiClient` is value-semantic
- **Symptom:** WebSocket/long-TCP handshake succeeds (101) but the first frame is RST
  (host side sees `ConnectionAbortedError` / `WinError 10053`).
- **Action:** after saving a client to a long-lived variable, never `stop()` the
  temporary copy; on ESP32 a copy shares the lwIP PCB and any copy's `stop()`/destructor
  resets the whole connection. Grep for `stop()` on saved copies before suspecting the
  protocol.
- **Anti-pattern:** assuming the copy releases only a local reference.

### R10. BLE peripherals must re-advertise on disconnect
- **Symptom:** device connects once, then vanishes forever after disconnecting.
- **Action:** register `onDisconnect` to flag re-advertise, and re-call
  `startAdvertising()` from `loop()` when `!connected && !advertising`. Explicitly build
  the advertisement payload; central-side re-scans must release the previous
  `BLEAdvertisedDevice*` first. Document that phone Bluetooth settings never list BLE
  peripherals (use nRF Connect / LightBlue).
- **Anti-pattern:** advertising only once in `setup()`.

### R11. Keep control signals out-of-band in binary transfers
- **Symptom:** binary transfer aborts at an offset that moves with the payload.
- **Action:** never scan payload bytes for a magic/control value (any byte appears with
  probability ~1/256). Use command lines, chunk timeouts, disconnect, or a separate
  endpoint for control. Mixed line/binary protocols need an explicit mode bit that makes
  the binary side read only timeout and length/EOF.
- **Anti-pattern:** in-band control-byte scanning inside data.

---

## Firmware structure / concurrency

### R12. Short ISRs only
- **Action:** clear-flag + signal (or push one ring-buffer item). `volatile` + critical
  section (or atomic) for variables shared with normal code. No `printf`/`malloc`/
  blocking I/O in the handler.
- **Anti-pattern:** doing real work in an ISR "because it's called often."

### R13. Bounded ring buffer over unbounded growth; real ticks over busy-loop time
- **Action:** decouple producer/consumer with a bounded ring buffer; track time with a
  real tick source (`millis()` / `esp_timer` / SysTick), never `counter += CONSTANT`.
  When implementing "take last N," verify the start-index math (`head + cap − count`) —
  the classic bug returns the oldest N.
- **Anti-pattern:** unbounded buffers and spin-loop wall-clock counters.

### R14. Large buffers on internal-RAM-constrained chips
- **Action:** allocate app-side buffers (> ~512 B) at runtime with the right
  `MALLOC_CAP_*` (PSRAM) instead of `static` arrays in internal RAM.
- **Anti-pattern:** a big static buffer silently failing a task-stack allocation and
  boot-looping the device.

### R15. Init once at the framework's boundary; initializer macros at declaration
- **Action:** services auto-started by the runtime must not be re-initialized
  (`ESP_ERR_INVALID_STATE` → warn + reuse). `*_DEFAULT()` macros are declaration-time
  initializers, not assignable to existing variables. Board setup (e.g.
  `usb_serial_jtag_driver_install()`) lives in one `board_init()`, not inline in
  `app_main()`.
- **Anti-pattern:** `ESP_ERROR_CHECK` around a duplicate init → guaranteed crash loop.

---

## Sensors / motion

### R16. Gyro drift needs continuous bias tracking, not a one-shot calibration
- **Symptom:** yaw drifts even after power-on auto-zero; warm-up reduces but doesn't
  stop it.
- **Action:** three-column log (`raw / bias / corrected`) to locate the drift source;
  if bias drifts with temperature, add **static detection** (corrected |gyro| sum below
  a threshold for ~1.5 s) plus per-tick low-pass bias tracking (small leak, e.g. 0.01)
  so bias follows zero. Add warm-up before sampling.
- **Anti-pattern:** lengthening the one-shot zero calibration; misreading register
  offsets when refactoring (re-verify each byte offset after a rewrite).

---

## Review / process

### R17. Correctness → maintainability → security → performance
- **Action:** review in that order; cite `file:line` + triggering scenario; tag severity
  `blocker/major/minor/nit`.
- **Anti-pattern:** style bikeshedding on a change that is behaviorally wrong.

### R18. Evidence over assertion
- **Action:** before claiming "done / works," produce the command and its actual output;
  mark un-run or un-verified steps explicitly. For a "before/after" code comparison,
  anchor the "before" to the exact commit SHA recorded at start (never `HEAD`, which has
  moved).
- **Anti-pattern:** "syntax parsed fine" or "process is alive" treated as "behavior
  correct."

### R19. One fix at a time for attribution
- **Action:** when a root-cause is unclear, change **one** variable, re-test, and only
  then proceed. If two straight attempts fail on the same symptom, stop and report.
- **Anti-pattern:** bundling several fixes and then being unable to attribute the cure.