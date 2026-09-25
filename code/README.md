# PNP — TMC2209 C1 spin firmware

Minimal ESP32-S3 firmware that spins the **C1** TMC2209 axis continuously
whenever the PCB is powered.

## Wiring (from `PNP.kicad_sch`, "TMC2209 C1" sheet)

| Signal | ESP32-S3 GPIO |
|--------|---------------|
| STEP   | GPIO6         |
| DIR    | GPIO5         |
| EN     | GPIO48 (active LOW) |
| UART TX/RX | GPIO18 / GPIO8 (unused) |

The C1 driver runs in **standalone STEP/DIR mode**. Motor current is set by the
driver module's onboard **VREF potentiometer** — if the motor has no torque or
doesn't move, turn VREF up. Microstep resolution is set by the module's MS1/MS2
pins (default ~1/8).

## Build & flash

PlatformIO core is installed under this machine's Python. From `code/`:

```bash
pio run                    # compile
pio run -t upload          # flash the connected board
pio device monitor         # optional: view serial output (115200)
```

Board auto-detects on the native USB port (`/dev/cu.usbmodem*`). If upload can't
find it, uncomment `upload_port` in `platformio.ini`.

## Tuning

Edit constants in `src/main.cpp`:

- `STEP_HZ` — step pulse frequency (higher = faster). 2000 Hz ≈ 75 RPM at 1/8
  microstepping on a 200-step motor.
- `DIR_LEVEL` — `HIGH`/`LOW` to reverse direction.
