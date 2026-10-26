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
- Right stick: C0 and C1 together; no selector or A-button switching.
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
acknowledgement, C selection (before pairing update), X precision, B stop, disconnect, page blur,
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


## Z top-zero limits and paired C controller motion

`pnp-zlimits-1` adds an adjustable Z soft travel limit. Open **Controller tuning
& saved positions** with the controller disabled. **Travel below top** is in
motor revolutions, not millimetres. `0` means no lower limit. A known top reference still enforces the upper
limit; without a top reference and with travel 0, limits are OFF. The source default is `Z_TRAVEL_TURNS_DEFAULT` in `src/main.cpp`;
a saved website setting overrides that default.

Setup: with no top reference and travel 0, jog carefully to the desired top
and disable the controller. Select **Set current Z as top (0)** to activate
the upper limit immediately. Optionally save a measured positive travel
below the top to add a bottom limit. Confirm **Positive motor direction is up** against the
actual machine (and the existing Reverse Z control). With a positive travel, the permitted logical
range is `-travel` through `0`; travel 0 permits any negative position. The travel and direction are saved to NVS;
the top reference is deliberately not saved. After restarting, configured
limits block Z until the top is established again. For unrestricted setup jogging, set travel to 0 and use **Clear top reference**.
Setting travel 0 alone preserves protection from a known top. Re-establish the reference
if the motor is moved while unpowered or loses steps; software position is
open-loop, without a physical homing switch or encoder.

Limited Z jog/spin commands use finite targets at the appropriate boundary,
allowing the pulse planner to decelerate before the target even if network
updates stall. The limits also reject manual relative and saved absolute moves
outside the range. Full-step / X precision changes rescale both position and
the boundary. Zero-all cannot silently redefine a limited Z reference, and
manual Z microstep changes are blocked while limits are enabled. Steps/rev
changes invalidate the top reference. Other axes remain usable when Z lacks
a reference. The diagnostic `/motiontest` emits pulses only with hardware EN
held OFF and restores its software positions.

Configuration API: `POST /zlimits?action=save&turns=VALUE&positiveUp=1`;
`POST /zlimits?action=top` sets the stationary current position to top zero;
`POST /zlimits?action=forget` clears the reference. Configuration requires
controller disabled and Z stopped. `/state` includes `zlimit`.

Xbox right-stick control now always sends identical C0 and C1 commands;
controller firmware rejects mismatched rates too. Individual manual controls
remain independent for tuning. LB always toggles the left vacuum and RB the
right, once per press (press again for OFF); old saved mappings cannot override
them. Controller must be enabled. The on-screen LB/RB buttons show the same
controls and text reports each vacuum's state. No real vacuum outputs were
switched during verification; those UI actions were tested with mocked requests.

Validation: host boundary tests cover upper/lower bounds, reversed direction,
fractional travel and 1/16-step scaling. Browser tests cover both C commands,
bumper toggling despite old saved mappings, no repeat while held, and mobile
layout. USB/OTA builds pass. Live tests check invalid settings, missing top
reference, blocked manual targets, top-bound controller commands at both
resolutions and rejection of unequal C commands, without requesting physical
travel. Limits are left OFF (`0`) with the top reference cleared and all drivers
OFF, ready for the user's measured range.

## October 24 controller default correction

`pnp-defaults-1` corrects the installed machine's vacuum sides: logical left
(LB) now uses GPIO4 (Q3/R12); logical right (RB) uses GPIO7 (Q1/R5). This swaps
the actual outputs while retaining consistent left/right labels in both manual
and controller views. Pumps remain OFF on startup.

Default X direction is reversed. All three controller speed groups (XY, Z, C)
now default to 12,000 in the existing 1/16-step-equivalent rate units. At full
stick and 100% jog speed this is 750 full-step pulses/s in normal mode and
3,000 pulses/s at 1/16 resolution in quarter-travel-speed precision mode.
A one-time browser migration applies the new speed/direction defaults to saved
profiles; later user tuning is preserved. Restore responsive defaults also
restores these values.

Validation: both firmware targets build, and simulated controller tests cover
profile migration, reversed X, speed scaling, coupled C motion and one toggle
per bumper press. The corrected physical vacuum sides need user observation;
no pump was energized as part of the automated checks.


## Top-zero enforcement repair

`pnp-toplimit-1` fixes the zero-only setup: Set current Z as top immediately
activates the upper bound, even with no lower travel configured. Previously
travel 0 disabled both boundaries despite topKnown=true. Negative/downward
positions remain unrestricted until a positive lower travel is saved. Clearing
the top reference disables top-only protection; with positive travel and no
reference, Z remains blocked pending a new zero.

Setting top zero now records the selected up direction atomically and preserves
any draft travel in the form. Unsaved changes are labelled, and the main
controller view reports OFF, TOP LIMIT ON, full limits ON, or Z BLOCKED based
on the board's saved settings. Top does not silently apply a draft bottom limit.

Validation: host boundary tests cover top-only behaviour, both signs, fractional
limits and precision resolution. Browser tests verify draft preservation, up
direction, applied-state feedback, successful save and failed-save preservation.
Live stationary tests verify that top-only trigger commands stop at zero with
travel 0; existing finite-range, manual-target and precision tests also pass.
USB and OTA builds succeeded. Test references were cleared and motors left OFF;
the user must physically re-establish top zero after the firmware restart.
