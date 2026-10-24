# PNP WiFi motor controller

ESP32-S3 firmware for six TMC2209 axes, Y0/Y1 pairing, vacuum outputs,
and optional gamepad jogging. Drivers and vacuums boot OFF.
Open http://pnp.local/ on the same network (fallback AP: PNP-Control,
http://192.168.4.1). WiFi and OTA credentials are in the ignored
`include/wifi_secrets.h` file.

## Wiring

All drivers share active-low EN on GPIO48. Individual driver enable uses
TMC2209 `toff`, so working UART communication is required.

| Axis | STEP | DIR | UART RX | UART TX through 1 kΩ | Address |
|---|---:|---:|---:|---:|---:|
| C1 | 6 | 5 | 8 | 18 | 1 |
| Z | 10 | 9 | 8 | 18 | 0 |
| X | 13 | 14 | 8 | 18 | 2 |
| Y0 | 2 | 1 | 39 | 40 | 2 |
| Y1 | 47 | 21 | 39 | 40 | 1 |
| C0 | 41 | 42 | 39 | 40 | 0 |

RX connects directly to the PDN bus. TX connects through R3 (bus A) or R1
(bus B). Assigning RX and TX to the same GPIO does not match this PCB.
Current and microstep settings are controlled over UART; they do not use
VREF or the standalone microstep defaults once configured.

## Build and OTA

From this directory:

```sh
pio run -e esp32s3
pio run -e ota -t upload
```

Use `--upload-port <board IP>` if mDNS discovery is unavailable. Do not
connect USB while 24 V is on; see the board power warning in `src/main.cpp`.
OTA stops pulse generation and disables the shared EN before flashing.

## Turn moves

Enable the required drivers, then select Run turns. A linked Y move requires
both axes active, connected to pulse channels, and enabled. Turn moves replace
existing motion, rather than extending an old target. Stop flushes the pulse
queue immediately; after an abrupt stop the physical position may need rehoming.

Pulses = turns × configured full steps/rev × selected microsteps.
For a 200-step motor at 16 microsteps, one turn is 3,200 pulses. With the
fallback speed of 400 Hz, allow about 8.2 seconds for one turn including
acceleration/deceleration. The fallback applies when slider speed is below
50 Hz. The run response reports the pulse count and actual command speed.

## October 21, 2026 turn-stop repair

- Corrected UART TX/RX assignments; all six UART checks changed from failed
  (`conn=2`) to successful (`conn=0`) on the connected board.
- FastAccelStepper 0.31.8 reproduced non-terminating RMT moves on Y1 and C0.
  The first four allocated channels (C1, Z, X, Y0) use MCPWM/PCNT; the next
  two use RMT. Y0 therefore stopped while its paired Y1 did not.
- Pinned upstream FastAccelStepper 0.31.7 commit
  `33b1f96e506ec48777d526c7bd22ed3e1a8bc053`. On this board, every axis and
  the linked Y pair stopped after exactly 320 pulses in the regression test.
  Rising edges were independently counted at the STEP GPIOs with EN HIGH;
  these are electrical pulse tests, not measurements of physical rotation.
- Added move return-code handling, pair preflight, turn bounds and valid
  microstep checks, full-step mapping (TMCStepper uses 0 for one full step),
  explicit STEP/DIR mode, and queue-flushing stops.
- The user also reported C1 now stops normally, with the gamepad disconnected.

`GET /state` reports firmware version, running/continuous flags, target,
and software position. `POST /motiontest` is an explicit diagnostic: it
stops everything, holds EN HIGH, emits 320 pulses per active channel and
then the Y pair, and reports GPIO pulse counts and completion. It restores
software coordinates and leaves every driver OFF. It blocks normal HTTP
handling for the duration of the short test. Do not use its result as
proof of mechanical travel or motor calibration.

## Xbox controller view

The main page now opens on the Xbox controls. Connect the controller, press a
button to expose it to the browser, center the sticks/release the triggers,
then select **Enable controller**. This prepares all six axes with the
controller tuning values and their existing current limits. Enable state is
never restored automatically on reload.

- Left stick: X and both Y motors. Triggers: Z.
- Right stick: C1 or C0; press A or use the on-screen selector to switch.
- Hold X for 1/16-step, quarter-speed precision; release it for full steps. The Jog speed slider scales motion
  speed without changing current or acceleration.
- B, Escape, or the sticky STOP button stops motion and disables controller
  input until enabled again. Disable controller also turns drivers OFF.
- LB/RB toggle the left/right vacuum. Stopping motion does not toggle vacuum.
- Controller tuning, custom stick/button mappings and saved positions are
  under the collapsed advanced section. Manual turn testing remains below.
  Tuning changes apply the next time the controller is enabled.

The browser reads the pad every animation frame, sends combined controller
updates over a persistent WebSocket at up to 25 Hz, and sends idle heartbeats every 100 ms. Only one update
can be awaiting acknowledgement; newer inputs replace unsent values instead of accumulating
requests. Each frame uses the latest complete stick state. The firmware applies both Y commands in that same request regardless
of the manual Y-link setting. It stops controller motion and invalidates the
enable session after 600 ms without a valid frame. Initial setup allows
up to two seconds for its HTTP reply, then requires a neutral first frame
before movement is accepted. Old frames cannot restart a
stopped session. Page blur, hiding the page and gamepad disconnection also send
STOP; a stalled response disables input in the browser.

`POST /controller?action=arm&mxy=1&mz=1&mc=1&axy=30000&az=20000&ac=20000`
prepares controller motion and returns an epoch. The persistent WebSocket on port 81 accepts
`sequence,epoch,x,y,z,c0,c1,precision` signed pulse-rate frames and acknowledges each
sequence. It validates the whole frame before applying it. The HTTP
`POST /control` endpoint remains available for diagnostics, but the browser
does not use HTTP for continuous jogging. `POST /controller?action=stop` stops and
turns every driver OFF. Manual tuning/moves are rejected while the controller
is enabled. Motion planning uses a 10 ms horizon with a 2 ms engine task;
`POST /motiontest?fast=1` tests 3,200 pulses per axis at up to 4,000 Hz with
hardware EN held OFF, in addition to the default 320-pulse test.

Validation of `pnp-controller-4`: USB and OTA builds passed. Simulated Xbox
browser tests covered neutral enable, simultaneous axes, one frame awaiting
acknowledgement, A selection, X precision, B stop, disconnect, page blur,
stalled responses and re-enabling after a session error. Live browser testing
held neutral input for eight seconds: 37 acknowledgements, 22 ms median and
421 ms maximum, with controller input remaining enabled. The earlier HTTP
transport measured 329 ms median. These are network acknowledgement times,
not measurements of physical motor response; WiFi delays still vary. Live
firmware checks covered expired sessions, invalid frames, socket disconnect,
the 600 ms timeout and exact 3,200-pulse stopping on all axes and the Y pair.
All drivers were turned OFF after verification.


## Full-step normal mode and X precision

`pnp-precision-1` boots with every axis at one full step per STEP pulse.
Controller enable also starts at full steps, including browsers that previously
saved 16-microstep controller profiles. Holding X requests actual 1/16
microstepping on all six drivers; releasing X returns to full stepping.
The precision bit is 0 or 1 in each WebSocket frame. HTTP diagnostic frames
can use `precision=0` or `precision=1`.

Resolution changes first decelerate all pulse channels, then update their UART
microstep registers at standstill and resume the latest stick command. A new
frame can cancel a pending change, and Stop cancels it without restarting.
Pulse rates and acceleration are scaled to the resolution so normal travel
speed stays comparable to the previous controller behavior. Precision keeps
one-quarter of normal travel speed. Advanced controller rates remain expressed
in 1/16-step equivalent units; manual motor controls display actual pulse Hz.
Controller resolution is fixed to 1 / 16; manual microstep selection remains
available when the controller is disabled.

Software positions are rescaled when resolution changes. New saved controller
positions use stable 1/16-step coordinates; existing saved values from the
previous default 16-microstep setup use those same units. Returning to full
steps quantizes positions to full-step boundaries.

Validation: both build targets passed; transition tests checked that UART
writes wait for standstill, cancellation uses the latest frame and Stop cannot
resume motion. Browser tests checked saved-profile migration, X precision,
rate scaling and the existing stop behavior. On the connected board, actual
TMC2209 register reads confirmed 1 -> 16 -> 1 microsteps on all six drivers,
with no STEP motion requested. All drivers were left OFF in full-step mode.
