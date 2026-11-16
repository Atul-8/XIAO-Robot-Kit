---
name: embedded-vibe-assistant
description: Maintainable embedded firmware (ESP-IDF / Arduino on ESP32) Vibe-coding and debugging companion: workflow, field-tested debug playbook, and code-quality guardrails. Use when writing, debugging, or reviewing embedded C/C++ firmware.
---

# Embedded Vibe Assistant

Write and debug **maintainable** embedded firmware — not just firmware that compiles.

This skill targets ESP-IDF and Arduino projects on ESP32-family microcontrollers
(e.g. the Seeed **XIAO ESP32-S3** robot-kit class of boards). It distills a
field-tested bug inventory (META rules) and the guardrails of three agent roles —
*embedded firmware engineer*, *code reviewer*, and *software architect* — into a
short, reusable operating procedure. Every command is reproducible on a clean
machine: no private paths, no tokens, no internal accounts.

## When to use this skill

Activate automatically when the task involves **embedded C / C++ firmware**, and
especially when:

- writing or modifying drivers, peripherals, ISRs, or BLE/WiFi/USB code;
- building with `idf.py` or `arduino-cli`;
- debugging a board that "compiles but misbehaves" (stalls, resets, silent drops);
- reviewing firmware for correctness, readability, and maintainability.

## Core doctrine

1. **Verify by running.** "Compiles" ≠ "works". Deliver the smallest change you can
   prove on the target, and state exactly what was verified and what was not.
2. **Evidence over assertion.** Report the command you ran and its actual output
   (log lines, sizes, hashes). Never claim "done" for an un-run step.
3. **Link-first.** Get the smallest end-to-end path working before expanding features.
4. **Diagnose before suspecting hardware.** Grab the boot/serial log first; most
   "dead board" symptoms are firmware bugs a few dozen ms after reset.
5. **Match existing conventions.** Grep neighboring files for include style, naming,
   and layering before adding new code. Consistency is a maintainability feature.

---

## Part A — The embedded development workflow

### A.1 Understand the build chain before writing code

| Toolchain | Build model | Entry point | Config source | Commands |
|-----------|-------------|-------------|---------------|----------|
| **ESP-IDF** | CMake + Ninja, component-based | each component's `CMakeLists.txt`, `main/` app | `sdkconfig` / `sdkconfig.defaults`, `partitions.csv` | `idf.py set-target`, `idf.py build`, `idf.py flash monitor` |
| **Arduino** | sketch compilation over Arduino core (which internally wraps ESP-IDF) | `.ino` sketch or `.cpp` via `arduino-cli` | board FQBN, `arduino-cli config` | `arduino-cli compile --fqbn <fqbn>`, `arduino-cli upload -p <port>` |

Before writing code, read the project's `sdkconfig`, `partitions.csv`,
`CMakeLists.txt`/`platformio.ini` (if present) and the board init. The chip model,
flash layout, and enabled components are **hard constraints**; do not invent them.

### A.2 The smallest loop that works

1. Confirm the toolchain works: build and flash an **unmodified** working example.
2. Make **one** change you can observe (one log line, one LED, one register read).
3. Build, flash, and observe that change — only then move on.
4. Expand incrementally. If two consecutive attempts fail on the same problem, stop
   and report the exact symptom instead of guessing further.

### A.3 Layering for maintainability

Keep a single direction of dependency:

```
drivers  (register/peripheral access, HAL)   ← no deps on app logic
   ↓
core     (device logic, state machines, math) ← testable without hardware
   ↓
interface (protocols, USB/BLE/WiFi, telemetry)
   ↓
app      (main / loop orchestration)
```

Rules:

- `drivers` never depends on app code. `app` may depend on everything below it.
- Shared code (constants, types, small utilities) owns **no mutable global state**.
- Cross-module runtime state goes through a **facade / interface**, never a stray
  `#define` or file-static variable duplicated across modules.
- Keep `app_main()` / `loop()` thin: delegate to well-named functions.

### A.4 ESP-IDF specifics

- **Initialize once, where the framework says.** Services already auto-started by
  the IDF runtime (e.g. TWDT / `esp_timer`) must not be re-initialized; a duplicate
  init returns `ESP_ERR_INVALID_STATE` and an `ESP_ERROR_CHECK` on it aborts the box
  into a crash loop. Handle the "already initialized" case by warning and reusing.
- **`*_DEFAULT()` macros are initializers, not constructors.** Do not assign one to an
  already-existing variable; it is meant for declaration-time member init.
- **Keep app code out of board init.** Board-setup calls (e.g.
  `usb_serial_jtag_driver_install()`) belong in one `board_init()`; do not inline them
  into `app_main()`.
- **Include prefix convention.** Library/component `.c` files include library headers
  with the `<component>/<header>.h` prefix; `.h` files include siblings relatively.
  Grep the existing `.c` files and match before writing new includes.
- **Internal-RAM-constrained builds:** large app-side buffers (roughly >512 B) belong
  in PSRAM (allocate at runtime with the right `MALLOC_CAP_*`), not as `static` arrays
  in internal RAM — a big static buffer can fail a task-stack allocation and boot-loop.
- **Parallel builds are capped.** On resource-limited hosts the ESP-IDF examples do not
  survive heavy parallel `idf.py build` (SIGTERM / exit 143 races). Build at most 2 in
  parallel, wait between batches, and fall back to serial.
- **A stalled ninja step is often a transient, not a bug.** `ldgen` / `pyparsing`
  parse errors and bootloader "Completed"-then-stall in incremental builds frequently
  pass on a plain re-run. Re-run once before changing any code.

### A.5 Arduino specifics

- **Always clean-compile before upload after editing source.**
  `arduino-cli compile --clean --fqbn <fqbn> <sketch>` then `upload`. `upload` alone
  reuses a stale `.bin` and reports "Hash verified" while flashing **old** bytes.
  Confirm the new firmware actually shipped by checking the compiled binary's size /
  digest changed.
- **Know which USB-CDC class you're on.** The ESP32-S3 Arduino core uses `HWCDC`
  (USB-Serial/JTAG). High-frequency `Serial.print` with no consumer open stalls the
  loop (see the playbook, P1). Design logging so the sketch survives "no monitor".
- **BLE peripherals must re-advertise.** Register `onDisconnect` and re-call
  `startAdvertising()` in `loop()` when `!connected && !advertising` (see playbook P2).
- **`WiFiClient` is value-semantic.** A copy shares the socket; `stop()` on any copy
  RSTs the whole connection (see playbook P3).

---

## Part B — Code-quality guardrails (apply to every change)

### B.1 Correctness first, then maintainability

Review in this order: **correctness → maintainability → security → performance**.
Report findings with `file:line` and the triggering scenario — never a bare opinion.
Use severity tags: `blocker` (must fix) / `major` (should fix) / `minor` (optional) /
`nit` (style — usually skip).

### B.2 Interrupt safety (non-negotiable)

- ISR does only: **clear flag + signal** (or push one item into a ring buffer).
- No `printf`, no `malloc`/`free`, no blocking I/O, no heavy math inside an ISR.
- Any variable shared between an ISR and normal code must be `volatile`; touching it
  across the boundary requires a critical section (interrupt-off) or an atomic.
- DMA buffers: keep the memory 4-byte aligned and in the region the controller can
  reach (watch cache coherency / `MALLOC_CAP_DMA`).

### B.3 Safe buffering and state

- **Ring buffer for stream data.** A bounded ring buffer decouples producer (ISR /
  high-frequency source) from consumer (task / loop). Never let a buffer grow
  unbounded, and never busy-poll.
- **No busy-wait for wall-clock time.** Track time with a tick counter, SysTick /
  `esp_timer`, or `millis()` — not `counter += CONSTANT` in a spin loop.
- **Idempotent init and single-owner lifecycle.** Guard against double-init; a
  destructor / `stop()` / `deinit` must release exactly what its init acquired.

### B.4 Readability conventions

- Name things by role, in the project's existing style; avoid single-letter names
  except loop indices.
- Replace magic numbers with named constants in **one** place.
- Header comments: units, value range, ownership (who frees), and thread-safety —
  not a restatement of the code.
- Keep functions small and single-purpose; keep `app_main()`/`loop()` thin.

---

## Part C — Field-tested debugging playbook

Work top-down; each entry is a concrete failure mode with the reproducible fix.

### P1. ESP32-S3 USB CDC `Serial` stalls the whole loop

**Symptom:** WiFi/BLE/TCP firmware stops responding while USB is plugged in but **no
serial monitor** is open; opening a monitor suddenly flushes everything.

**Cause:** `HWCDC` busy-waits flushing the USB FIFO when the host sends no IN token.
Any `Serial.write`, `if (Serial)`, or `operator bool` triggers it while
`isCDC_Connected()` is true.

**Fix (choose one):**
1. Run the product on battery (USB unplugged → `isPlugged()==false` → no stall).
2. Always keep a serial monitor open during development.
3. Add a compile-time macro to disable loop-level `Serial.print` in release builds.

**Note:** evaluating `if (Serial)` itself can block — that guard is only safe when USB
is unplugged. When USB-CDC firmware suddenly stops talking wirelessly, first test
"does opening a serial monitor restore it?" before touching protocol code.

### P2. BLE peripheral disappears after one disconnect

**Symptom:** phone connects once, disconnects, and the device is never seen again
until reset.

**Cause:** advertising stops on connection and does **not** auto-resume on disconnect.

**Fix:**
- Set a `BLEServerCallbacks::onDisconnect` that flags "advertise again".
- In `loop()`, about once per second: `if (!connected && !advertising) startAdvertising();`.
- Build the advertisement explicitly (flags `0x06` + 128-bit ServiceUUID in the main
  payload, full device name in the scan response); print "advertising" once at start.
- On the central side, release the previous `BLEAdvertisedDevice*` handle before every
  re-scan, and clear it on failed connect.
- Document that phone OS **Bluetooth settings** never lists BLE peripherals — users must
  use nRF Connect / LightBlue, or "can't find it" is misdiagnosed as hardware failure.

### P3. WebSocket / long TCP: handshake OK but first frame RST

**Symptom:** client completes a handshake (HTTP 101) and the very first frame is reset
(`ConnectionAbortedError` / `WinError 10053` on the PC side).

**Cause:** `WiFiClient ws = server.available(); saved = ws; … ws.stop();` — `WiFiClient`
is value-semantic; `saved = ws` shares the same lwIP PCB, and `stop()` on the temporary
copy tears down the socket.

**Fix:** after a client is saved to a long-lived variable, **never** `stop()` the
temporary copy. Use a flag to distinguish short-lived vs. long-lived connections and
only `stop()` the short-lived ones. When diagnosing "first frame RST," grep for
`stop()` on saved client copies before suspecting protocol/network issues.

### P4. Arduino upload "succeeds" but runs old code

**Symptom:** `arduino-cli upload` prints "Hash verified" yet the board shows old
fields; binary size unchanged from before the edit.

**Cause:** upload reused the old build cache.

**Fix:** `arduino-cli compile --clean --fqbn <fqbn> <sketch>` before `upload`. If
`--clean` occasionally drops `partitions.bin` ("No such file or directory"), run one
plain `compile` to regenerate it. Verify by changed binary size + actual serial output.

### P5. Stuck after flashing / boot loop (ESP32-S3, USB-Serial/JTAG)

**Symptom:** flash reports success but the device hangs in download mode or sits on a
ROM console (PID_1001), or endlessly reboots.

**Fix order:**
1. **Grab the boot/serial log first** — most cases are a firmware abort ~tens of ms
   after reset, visible in one screen of log. Do not suspect USB/drivers/hardware before
   seeing the log.
2. If it's a crash loop: look for a duplicate init of an auto-started IDF service
   (`ESP_ERR_INVALID_STATE`) and degrade gracefully instead of aborting.
3. To actually boot after flashing over USB-Serial/JTAG: `esptool … --after watchdog-reset`
   (a soft "Hard resetting via RTS" does **not** reset the chip on USJ). Verify reset by
   the boot banner / a new COM port, not by the console text.
4. Before suspecting the board/PSRAM/flash, flash a known-good image (e.g. a stock
   blink example) to prove the hardware chain — one command, big diagnostic value.

### P6. Binary protocol aborts at a "random" offset

**Symptom:** a firmware image / binary transfer aborts partway, at an offset that
changes with the payload.

**Cause:** in-band control-byte scanning (e.g. treating `0x1A` as abort inside the data
stream). Any byte value eventually appears in binary data (~1/256 per byte).

**Fix:** keep control signals **out-of-band** — command lines, chunk timeouts, connection
close, or a separate endpoint. Never scan payload bytes for a magic value. Mixed
line/binary protocols need an explicit mode bit: in binary mode the parser sees only
timeout and length/EOF.

### P7. Serial port is a shared exclusive resource

**Symptom:** user reports `PermissionError` / "port busy", or a monitor can't open.

**Cause:** a diagnostic command (often a `monitor` you ran) still holds the port via a
surviving child process, or you're racing host enumeration.

**Fix:**
- Before any read-only diagnostic on a port/probe, assume it blocks the user; prefer
  reading logs/files/offline artifacts first.
- A `timeout`/`kill` on the parent does **not** release the port — verify the child
  process is gone and confirm release.
- When the user reports a busy port, check your own leftover processes first.

### P8. State that "looks alive" but isn't coherent

- **Time/clock:** never simulate wall-clock with `counter += CONSTANT` in a loop — use a
  real tick source.
- **Ring buffer "take last N":** double-check the start index math (head + cap − count).
  The classic bug returns the *oldest* N instead of the *newest* N.
- **Latched values:** a status indicator must show *current* trust level, not a one-shot
  sample taken once at boot.

---

## Part D — Definition of done (self-check before claiming anything)

- [ ] Code compiles with the project's toolchain (`idf.py build` or
      `arduino-cli compile --clean --fqbn …`).
- [ ] The change was flashed/run and its observable effect was **seen** (log / LED /
      register read / host-side signal) — or explicitly marked "not verified".
- [ ] No ISR does blocking work; shared variables are `volatile` / atomically accessed.
- [ ] Dependencies point downward (drivers ← core ← interface ← app); no magic numbers;
      cross-module state goes through a facade.
- [ ] Include style matches neighboring files; new files follow the project's layering.
- [ ] The exact command + output that proves each "done" claim is reported.

For the full rule catalog behind this skill, see
`references/field-tested-rules.md`.