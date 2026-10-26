/*
 * PNP board — 6-axis TMC2209 tuning + turn-test panel over WiFi.
 *
 * Target: ESP32-S3-DevKitC (on the PNP PCB)
 *
 * Power from the 24 V source, join WiFi AP "PNP-Control", open http://192.168.4.1.
 * No USB needed while running (avoids the 24V-vs-USB 3.3V contention). Flash with
 * USB only while the 24 V is OFF.
 *
 * Axes (traced from PNP.kicad_pcb). EN = GPIO48 (global, held low; per-axis on/off
 * via TMC `toff`). Two UART buses; address set by each socket's MS1/MS2 straps:
 *   Bus A RX=GPIO8, TX=GPIO18 (1k series R):  C1=addr1(step6,dir5)  Z=addr0(step10,dir9)  X=addr2(step13,dir14)
 *   Bus B RX=GPIO39, TX=GPIO40 (1k series R): Y0=addr2(step2,dir1) Y1=addr1(step47,dir21) C0=addr0(step41,dir42)
 *
 * Step/dir pulses: FastAccelStepper (hardware-timed, with acceleration + bounded
 * "move N turns"). Current/microstep/chopper: TMC2209 over UART.
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <limits.h>
#include <errno.h>
#include <WebSocketsServer.h>
#include "wifi_secrets.h"

// ---- WiFi ----
// Joins your network (WIFI_SSID/WIFI_PASS from wifi_secrets.h). If it can't
// connect within the timeout, it falls back to its own AP so you're never
// locked out. OTA (over-the-air) firmware updates are enabled either way.
static constexpr char AP_SSID[] = "PNP-Control";   // fallback AP name
static constexpr char AP_PASS[] = "pnp12345";      // fallback AP pass (>= 8 chars)
static constexpr uint32_t WIFI_JOIN_TIMEOUT_MS = 12000;
static constexpr char OTA_HOSTNAME[] = "pnp";

// ---- UART buses ----
static constexpr int      PIN_UART_TX_A = 18;
static constexpr int      PIN_UART_TX_B = 40;
static constexpr int      PIN_UART_A = 8;        // C1, Z, X
static constexpr int      PIN_UART_B = 39;       // Y0, Y1
static constexpr uint32_t UART_BAUD  = 115200;
HardwareSerial TMCSerialA(1);
HardwareSerial TMCSerialB(2);

static constexpr float R_SENSE = 0.11f;          // **match your driver module**
static constexpr int   PIN_EN  = 48;             // global enable, active LOW

// ---- Vacuum low-side switches (active HIGH; 10k gate pulldowns keep them OFF) ----
static constexpr int PIN_VAC_LEFT  = 4;          // Q3 / R12: installed machine left vacuum
static constexpr int PIN_VAC_RIGHT = 7;          // Q1 / R5: installed machine right vacuum
bool vacLeft = false, vacRight = false;
bool yLinked = false;                            // Y0 & Y1 move together when true

// ---- Drivers (Stream*, R_SENSE, address) ----
TMC2209Stepper drvC1(&TMCSerialA, R_SENSE, 0b01);
TMC2209Stepper drvZ (&TMCSerialA, R_SENSE, 0b00);
TMC2209Stepper drvX (&TMCSerialA, R_SENSE, 0b10);
TMC2209Stepper drvY0(&TMCSerialB, R_SENSE, 0b10);
TMC2209Stepper drvY1(&TMCSerialB, R_SENSE, 0b01);
TMC2209Stepper drvC0(&TMCSerialB, R_SENSE, 0b00);

FastAccelStepperEngine engine = FastAccelStepperEngine();

// ---- Per-axis state ----
struct Axis {
  const char*     name;
  TMC2209Stepper* drv;
  int             stepPin;
  int             dirPin;
  FastAccelStepper* fs;      // assigned at connect; NULL if no free channel
  // settings
  bool            active;    // present/usable (gated in settings panel)
  // live runtime
  uint16_t        current_mA;
  uint16_t        maxCurrent; // per-motor RMS cap (mA) = rated peak / sqrt(2)
  uint32_t        speed_hz;
  uint32_t        accel;     // steps/s^2
  uint16_t        microsteps;
  bool            spread;    // true=spreadCycle, false=stealthChop
  bool            dir;       // true=forward
  bool            driverOn;  // TMC toff on/off
  uint8_t         conn;      // test_connection(), 0=OK
  bool            spinning;  // true only while running continuously (Spin ∞)
};

#define NAX 6
Axis axes[NAX] = {
  //name  drv     step dir  fs  active cur   max   spd acc   mic sprd dir  on  conn   motor (rated peak/phase)
  {"C1", &drvC1,  6,   5,   nullptr, true,  340,  424, 0, 2000, 1, true, true, false, 0}, // Nema8 hollow 0.6A
  {"Z",  &drvZ,  10,   9,   nullptr, true,  1200, 1485, 0, 2000, 1, true, true, false, 0}, // Nema17 2.1A
  {"X",  &drvX,  13,  14,   nullptr, false, 1200, 1485, 0, 2000, 1, true, true, false, 0}, // Nema17 2.1A
  {"Y0", &drvY0,  2,   1,   nullptr, false, 1200, 1485, 0, 2000, 1, true, true, false, 0}, // Nema17 2.1A
  {"Y1", &drvY1, 47,  21,   nullptr, false, 1200, 1485, 0, 2000, 1, true, true, false, 0}, // Nema17 2.1A
  {"C0", &drvC0, 41,  42,   nullptr, false, 340,  424, 0, 2000, 1, true, true, false, 0}, // Nema8 hollow 0.6A
};

// ---- Global settings (slider ranges etc.), persisted to NVS ----
struct Settings {
  uint16_t curMin, curMax;
  uint16_t spdMin, spdMax;
  uint32_t accMin, accMax;
  uint16_t turnMin, turnMax;
  uint16_t stepsPerRev;      // full steps/rev of the motor (1.8deg => 200)
};
Settings cfg = {0, 1500, 0, 4000, 100, 50000, 0, 100, 200};

// User-tunable Z travel in MOTOR revolutions below the top; 0 disables limits.
// No millimetre conversion is assumed without the lead-screw/belt calibration.
static constexpr float Z_TRAVEL_TURNS_DEFAULT = 0.0f;
static float zTravelTurns = Z_TRAVEL_TURNS_DEFAULT;
static bool zPositiveUp = true, zTopKnown = false;
static constexpr int Z_AXIS = 1;

Preferences prefs;
WebServer server(80);
WebSocketsServer controlSocket(81);

// ---- Persistence ----
static void loadSettings() {
  prefs.begin("pnp", true);
  cfg.curMin = prefs.getUShort("curMin", cfg.curMin);
  cfg.curMax = prefs.getUShort("curMax", cfg.curMax);
  cfg.spdMin = prefs.getUShort("spdMin", cfg.spdMin);
  cfg.spdMax = prefs.getUShort("spdMax", cfg.spdMax);
  cfg.accMin = prefs.getULong("accMin", cfg.accMin);
  cfg.accMax = prefs.getULong("accMax", cfg.accMax);
  cfg.turnMin = prefs.getUShort("turnMin", cfg.turnMin);
  cfg.turnMax = prefs.getUShort("turnMax", cfg.turnMax);
  cfg.stepsPerRev = prefs.getUShort("spr", cfg.stepsPerRev);
  for (int i = 0; i < NAX; ++i)
    axes[i].active = prefs.getBool((String("act") + axes[i].name).c_str(), axes[i].active);
  yLinked = prefs.getBool("yLink", yLinked);
  zTravelTurns = prefs.getFloat("zTravel", Z_TRAVEL_TURNS_DEFAULT);
  zPositiveUp = prefs.getBool("zUp", true);
  if (!isfinite(zTravelTurns) || zTravelTurns < 0 || (double)zTravelTurns * cfg.stepsPerRev > INT32_MAX / 256) zTravelTurns = 0;
  prefs.end();
}

static void saveSettings() {
  prefs.begin("pnp", false);
  prefs.putUShort("curMin", cfg.curMin); prefs.putUShort("curMax", cfg.curMax);
  prefs.putUShort("spdMin", cfg.spdMin); prefs.putUShort("spdMax", cfg.spdMax);
  prefs.putULong("accMin", cfg.accMin);  prefs.putULong("accMax", cfg.accMax);
  prefs.putUShort("turnMin", cfg.turnMin); prefs.putUShort("turnMax", cfg.turnMax);
  prefs.putUShort("spr", cfg.stepsPerRev);
  for (int i = 0; i < NAX; ++i)
    prefs.putBool((String("act") + axes[i].name).c_str(), axes[i].active);
  prefs.putBool("yLink", yLinked);
  prefs.putFloat("zTravel", zTravelTurns); prefs.putBool("zUp", zPositiveUp);
  prefs.end();
}

// ---- Driver / motion control ----
static void connectStepper(Axis& a) {
  if (a.fs) return;
  a.fs = engine.stepperConnectToPin(a.stepPin);
  if (a.fs) {
    a.fs->setDirectionPin(a.dirPin);
    a.fs->setForwardPlanningTimeInMs(10); // 5 task cycles of margin at 2 ms
    a.fs->setAutoEnable(false);          // EN is shared, managed manually
    a.fs->setAcceleration(a.accel);
    a.fs->setSpeedInHz(max<uint32_t>(a.speed_hz, 1));
  }
}

static void initDriver(TMC2209Stepper* d) {
  d->begin();
  d->VACTUAL(0);             // use external STEP/DIR, never internal velocity mode
  d->pdn_disable(true);
  d->I_scale_analog(false);   // ignore VREF pot; current is software-defined
  d->mstep_reg_select(true);
  d->pwm_autoscale(true);
}

// Push current/microstep/chopper/enable to the TMC.
static void applyDriver(Axis& a) {
  a.drv->rms_current(a.current_mA);
  a.drv->microsteps(a.microsteps == 1 ? 0 : a.microsteps);
  a.drv->en_spreadCycle(a.spread);
  a.drv->toff(a.driverOn ? 4 : 0);
}

static bool validMicrosteps(long v) {
  return v >= 1 && v <= 256 && (v & (v - 1)) == 0;
}

static uint32_t turnSpeed(const Axis& a) {
  return a.speed_hz < 50 ? 400 : a.speed_hz;
}

// forceStop() drains queued commands and can remain stuck in a failed backend.
// Flush the hardware queue for an explicit stop; an abrupt stop loses exact position.
static void stopAxis(Axis& a) {
  if (a.fs) a.fs->forceStopAndNewPosition(a.fs->getCurrentPosition());
  a.spinning = false;
}

static int32_t turnsToSteps(float turns, const Axis& a) {
  return (int32_t)lround(turns * (float)cfg.stepsPerRev * (float)a.microsteps);
}

static bool zLimitsEnabled() { return zTravelTurns > 0 || zTopKnown; }
static int32_t zBottomSteps() {
  return (int32_t)floor((double)zTravelTurns * cfg.stepsPerRev * axes[Z_AXIS].microsteps);
}
static bool zTargetAllowed(int64_t rawTarget) {
  if (!zLimitsEnabled()) return true;
  if (!zTopKnown) return false;
  const int64_t logical = zPositiveUp ? rawTarget : -rawTarget;
  return logical <= 0 && (zTravelTurns <= 0 || logical >= -(int64_t)zBottomSteps());
}
static int8_t startZRate(int32_t hz) {
  Axis& a = axes[Z_AXIS];
  if (!hz) { a.fs->stopMove(); return MOVE_OK; }
  if (!zLimitsEnabled()) return hz > 0 ? a.fs->runForward() : a.fs->runBackward();
  if (!zTopKnown) { a.fs->stopMove(); return -1; }
  // A finite target lets the pulse planner brake before the limit, even if
  // WiFi or the main loop stalls. No polling-based emergency boundary stop.
  const bool up = (hz > 0) == zPositiveUp;
  if (!up && zTravelTurns <= 0) return hz > 0 ? a.fs->runForward() : a.fs->runBackward();
  return a.fs->moveTo(up ? 0 : (zPositiveUp ? -zBottomSteps() : zBottomSteps()));
}

static int8_t spinContinuous(Axis& a) {
  if (!a.fs || !a.driverOn || a.speed_hz < 1) { if (a.fs) a.fs->stopMove(); a.spinning = false; return -1; }
  a.fs->setAcceleration(a.accel);
  a.fs->setSpeedInHz(a.speed_hz);
  int8_t result = (&a == &axes[Z_AXIS]) ? startZRate(a.dir ? a.speed_hz : -(int32_t)a.speed_hz) : (a.dir ? a.fs->runForward() : a.fs->runBackward());
  a.spinning = result == MOVE_OK;
  return result;
}

// Re-apply speed/direction to a motor that is ALREADY spinning — never starts it.
static void adjustIfSpinning(Axis& a) {
  if (!a.fs || !a.spinning) return;
  a.fs->setSpeedInHz(max<uint32_t>(a.speed_hz, 1));
  if (&a == &axes[Z_AXIS]) startZRate(a.dir ? a.speed_hz : -(int32_t)a.speed_hz);
  else if (a.dir) a.fs->runForward(); else a.fs->runBackward();
}

static int8_t runTurns(Axis& a, float turns) {
  if (!a.fs || !a.driverOn) return -1;
  // A turns command replaces any prior motion, starting from the current position.
  stopAxis(a);
  a.fs->setAcceleration(a.accel);
  // If no usable speed is set, run the move at a sane default instead of
  // crawling at 1 Hz (which looks like it never finishes).
  a.fs->setSpeedInHz(turnSpeed(a));
  int32_t steps = turnsToSteps(turns, a);
  if (!a.dir) steps = -steps;
  if (&a == &axes[Z_AXIS] && !zTargetAllowed((int64_t)a.fs->getCurrentPosition() + steps)) return -1;
  return a.fs->move(steps);
}

// One latest controller frame drives every axis; no per-axis HTTP queue.
static uint8_t controllerSocketOwner = 255;
static bool controllerArmed = false, controllerPrimed = false;
static uint32_t controllerEpoch = 1, controllerLastFrame = 0;
static constexpr uint32_t CONTROLLER_TIMEOUT_MS = 600;
static int32_t controllerHz[NAX] = {};
static long controllerDesiredHz[NAX] = {};
static uint32_t controllerAccel16[NAX] = {};
static bool controllerPrecision = false, controllerModeChanging = false;

static void releaseController(bool disableDrivers = false) {
  controllerArmed = false; controllerPrimed = false; controllerModeChanging = false;
  controllerSocketOwner = 255;
  ++controllerEpoch;  // queued frames from a previous enable can never restart motion
  for (int i = 0; i < NAX; ++i) {
    stopAxis(axes[i]); controllerHz[i] = 0;
    if (disableDrivers) { axes[i].driverOn = false; axes[i].drv->toff(0); }
  }
}

static bool integerArg(const char* key, long low, long high, long& value) {
  const String raw = server.arg(key);
  char* end = nullptr;
  errno = 0;
  value = strtol(raw.c_str(), &end, 10);
  return errno != ERANGE && raw.length() && end != raw.c_str() && *end == '\0' && value >= low && value <= high;
}

static bool controllerResolution(bool precision) {
  const uint16_t micro = precision ? 16 : 1;
  int32_t positions[NAX];
  for (int i = 0; i < NAX; ++i) {
    const auto& a = axes[i];
    if (!a.fs || a.fs->isRunning()) return false;
    const int64_t scaled = (int64_t)a.fs->getCurrentPosition() * micro / a.microsteps;
    if (scaled < INT32_MIN || scaled > INT32_MAX) return false;
    positions[i] = (int32_t)scaled;
  }
  for (int i = 0; i < NAX; ++i) {
    auto& a = axes[i];
    a.drv->microsteps(micro == 1 ? 0 : micro);
    a.fs->setCurrentPosition(positions[i]);
    a.microsteps = micro;
    a.accel = max<uint32_t>(1, controllerAccel16[i] * micro / 16);
    a.fs->setAcceleration(a.accel);
  }
  controllerPrecision = precision;
  return true;
}

static void handleController() {
  if (server.arg("action") == "stop") {
    releaseController(true);
    server.send(200, "application/json", "{\"armed\":0}"); return;
  }
  if (server.arg("action") != "arm") {
    server.send(400, "text/plain", "Unknown controller action"); return;
  }
  long micro[3], accel[3];
  const char* mk[] = {"mxy", "mz", "mc"};
  const char* ak[] = {"axy", "az", "ac"};
  for (int g = 0; g < 3; ++g) {
    if (!integerArg(mk[g], 1, 256, micro[g]) || micro[g] != 1 ||
        !integerArg(ak[g], 100, cfg.accMax, accel[g])) {
      server.send(400, "text/plain", "Invalid controller microsteps or acceleration"); return;
    }
  }
  releaseController(true);
  digitalWrite(PIN_EN, HIGH);
  // Prepare all axes before enabling any of them. Current limits are unchanged.
  for (int i = 0; i < NAX; ++i) {
    auto& a = axes[i];
    const int g = (i == 0 || i == 5) ? 2 : (i == 1 ? 1 : 0);
    a.active = true; connectStepper(a); initDriver(a.drv);
    a.conn = a.drv->test_connection();
    if (!a.fs || a.conn != 0) {
      releaseController(true);
      server.send(409, "text/plain", String(a.name) + ": controller setup failed"); return;
    }
    controllerAccel16[i] = accel[g];
    // Microsteps are set for all axes together below, while stopped.
    a.accel = max<uint32_t>(1, accel[g] / 16);
    a.fs->setAcceleration(a.accel); applyDriver(a);
  }
  if (!controllerResolution(false)) {
    releaseController(true); server.send(409, "text/plain", "Cannot set full-step mode"); return;
  }
  for (auto& a : axes) { a.driverOn = true; a.drv->toff(4); }
  digitalWrite(PIN_EN, LOW);
  controllerArmed = true; controllerLastFrame = millis();
  server.send(200, "application/json", "{\"armed\":1,\"epoch\":" + String(controllerEpoch) + "}");
}

static const char* applyControlSpeeds(const long requested[NAX], bool precision = false) {
  // Setup uses HTTP; allow its reply to arrive before the first neutral handshake.
  if (!controllerPrimed) {
    for (int i = 0; i < NAX; ++i) if (requested[i] != 0) {
      releaseController(); return "Center controls before starting the controller";
    }
    controllerPrimed = true;
  }
  for (const auto& a : axes)
    if (!a.active || !a.driverOn || !a.fs) { releaseController(); return "Controller axis unavailable"; }
  if (requested[0] != requested[5]) return "C0 and C1 controller speeds must match";
  for (int i = 0; i < NAX; ++i) controllerDesiredHz[i] = requested[i];
  controllerLastFrame = millis();
  if (precision != controllerPrecision) {
    if (!controllerModeChanging) {
      for (int i = 0; i < NAX; ++i) { axes[i].fs->stopMove(); controllerHz[i] = 0; }
      controllerModeChanging = true;
    }
    return nullptr;
  }
  // Releasing X before the stop completes cancels the resolution change.
  controllerModeChanging = false;
  for (int i = 0; i < NAX; ++i) {
    auto& a = axes[i]; const int32_t hz = (i == Z_AXIS && zLimitsEnabled() && !zTopKnown) ? 0 : requested[i], old = controllerHz[i];
    if (hz == old) continue;
    a.spinning = false;
    if (hz == 0) a.fs->stopMove();
    else {
      a.fs->setSpeedInHz(abs(hz));
      int8_t result = MOVE_OK;
      if ((hz > 0 && old > 0) || (hz < 0 && old < 0)) {
        // Update speed without repeatedly restarting the continuous ramp.
        a.fs->applySpeedAcceleration();
      } else result = i == Z_AXIS ? startZRate(hz) : (hz > 0 ? a.fs->runForward() : a.fs->runBackward());
      if (result != MOVE_OK) {
        releaseController(); return "Controller move rejected";
      }
    }
    controllerHz[i] = hz;
  }
  controllerLastFrame = millis();
  return nullptr;
}

// Drain the pulse queues before changing UART microstep resolution.
// The latest incoming frame remains cached, so held sticks resume after the change.
static void serviceControllerResolution() {
  if (!controllerArmed || !controllerModeChanging) return;
  for (const auto& a : axes) if (a.fs->isRunning()) return;
  if (!controllerResolution(!controllerPrecision)) { releaseController(); return; }
  controllerModeChanging = false;
  if (applyControlSpeeds(controllerDesiredHz, controllerPrecision)) releaseController();
}

static void handleControlFrame() {
  server.client().setNoDelay(true);
  long epoch;
  if (!integerArg("epoch", 1, LONG_MAX, epoch) || !controllerArmed || (uint32_t)epoch != controllerEpoch) {
    server.send(409, "text/plain", "Controller disabled; enable it again"); return;
  }
  const char* keys[NAX] = {"c1", "z", "x", "y", "y", "c0"};
  long requested[NAX];
  // Validate the complete frame before touching a motor.
  for (int i = 0; i < NAX; ++i) {
    if (!integerArg(keys[i], -(long)cfg.spdMax, cfg.spdMax, requested[i])) {
      releaseController(); server.send(400, "text/plain", "Invalid controller speed"); return;
    }
    if (!axes[i].active || !axes[i].driverOn || !axes[i].fs) {
      releaseController(); server.send(409, "text/plain", "Controller axis unavailable"); return;
    }
  }
  long precision = 0;
  if (server.hasArg("precision") && !integerArg("precision", 0, 1, precision)) {
    releaseController(); server.send(400, "text/plain", "Invalid precision mode"); return;
  }
  const char* error = applyControlSpeeds(requested, precision != 0);
  if (error) server.send(409, "text/plain", error);
  else server.send(200, "application/json", "{\"ok\":1}");
}

static void controlSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_DISCONNECTED) {
    if (client == controllerSocketOwner) releaseController();
    return;
  }
  if (type != WStype_TEXT) return;
  // CSV frame: sequence, epoch, X, Y pair, Z, C0, C1, precision. Bounded, exact parse.
  long fields[8];
  bool valid = length > 0 && length < 96;
  char buffer[96];
  if (valid) { memcpy(buffer, payload, length); buffer[length] = '\0'; }
  char* next = buffer;
  for (int i = 0; valid && i < 8; ++i) {
    char* end = nullptr; errno = 0;
    fields[i] = strtol(next, &end, 10);
    const long low = i < 2 ? 1 : (i == 7 ? 0 : -(long)cfg.spdMax);
    const long high = i < 2 ? LONG_MAX : (i == 7 ? 1 : cfg.spdMax);
    valid = errno != ERANGE && end != next && fields[i] >= low && fields[i] <= high &&
            (i < 7 ? *end == ',' : end == buffer + length);
    next = end + (i < 7 ? 1 : 0);
  }
  if (!valid) {
    if (client == controllerSocketOwner) releaseController();
    controlSocket.sendTXT(client, "{\"error\":\"Invalid controller frame\"}"); return;
  }
  if (!controllerArmed || (uint32_t)fields[1] != controllerEpoch) {
    controlSocket.sendTXT(client, "{\"error\":\"Controller disabled; enable it again\"}"); return;
  }
  const long requested[NAX] = {fields[6], fields[4], fields[2], fields[3], fields[3], fields[5]};
  const char* error = applyControlSpeeds(requested, fields[7] != 0);
  if (error) { controlSocket.sendTXT(client, String("{\"error\":\"") + error + "\"}"); return; }
  controllerSocketOwner = client;
  controlSocket.sendTXT(client, String("{\"ok\":1,\"seq\":") + fields[0] + "}");
}

static void handleZLimits() {
  if (controllerArmed || (axes[Z_AXIS].fs && axes[Z_AXIS].fs->isRunning())) {
    server.send(409, "text/plain", "Disable controller and stop Z before setting limits"); return;
  }
  const String action = server.arg("action");
  if (action == "forget") { zTopKnown = false; server.send(200, "text/plain", "Z top reference cleared"); return; }
  if (action == "top") {
    if (!axes[Z_AXIS].fs) { server.send(409, "text/plain", "Activate Z first"); return; }
    // Record the selected up direction together with the physical top reference.
    // Do not silently apply the draft travel distance; Save remains explicit.
    long up = zPositiveUp ? 1 : 0;
    if (server.hasArg("positiveUp") && !integerArg("positiveUp", 0, 1, up)) {
      server.send(400, "text/plain", "Invalid Z up direction"); return;
    }
    if (zPositiveUp != (up != 0)) { zPositiveUp = up != 0; saveSettings(); }
    axes[Z_AXIS].fs->setCurrentPosition(0); zTopKnown = true;
    server.send(200, "text/plain", "Z top zero set; upward limit is ACTIVE"); return;
  }
  if (action != "save") { server.send(400, "text/plain", "Unknown Z limit action"); return; }
  const String raw = server.arg("turns"); char* end = nullptr;
  const float travel = strtof(raw.c_str(), &end); long up;
  if (!raw.length() || end == raw.c_str() || *end != '\0' || !isfinite(travel) || travel < 0 ||
      (travel > 0 && (double)travel * cfg.stepsPerRev < 1) ||
      (double)travel * cfg.stepsPerRev > INT32_MAX / 256 ||
      !integerArg("positiveUp", 0, 1, up)) {
    server.send(400, "text/plain", "Enter 0 to disable, or a positive travel of at least one full motor step"); return;
  }
  const bool sameDirection = zPositiveUp == (up != 0);
  if (travel > 0 && zTopKnown && sameDirection) {
    const int64_t logical = (int64_t)axes[Z_AXIS].fs->getCurrentPosition() * (zPositiveUp ? 1 : -1);
    const int64_t bottom = (int64_t)floor((double)travel * cfg.stepsPerRev * axes[Z_AXIS].microsteps);
    if (logical > 0 || logical < -bottom) {
      server.send(409, "text/plain", "Current Z is outside this range; move inside it before saving"); return;
    }
  }
  if (!sameDirection) zTopKnown = false;
  zTravelTurns = travel; zPositiveUp = up != 0; saveSettings();
  server.send(200, "text/plain", zTravelTurns > 0 ? (zTopKnown ? "Z top and bottom limits active" : "Z range saved; set top zero before moving Z") : (zTopKnown ? "Z top limit active; lower limit not set" : "Z limits disabled"));
}

// ---- Web UI ----
static const char PAGE[] PROGMEM = R"rawhtml(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PNP Controller</title>
<link rel="icon" href="data:,">
<style>
 :root{color-scheme:dark}
 body{font-family:system-ui,-apple-system,sans-serif;margin:0;background:#0f1115;color:#e6e6e6}
 header{padding:14px 18px;background:#171a21;border-bottom:1px solid #2a2e37;display:flex;justify-content:space-between;align-items:center}
 header b{font-size:19px}
 button{background:#22262f;color:#e6e6e6;border:1px solid #2a2e37;border-radius:8px;padding:8px 12px;font-size:14px;cursor:pointer}
 .wrap{display:flex;flex-wrap:wrap;gap:14px;padding:14px}
 .card{background:#171a21;border:1px solid #2a2e37;border-radius:12px;padding:14px;flex:1;min-width:290px}
 .card h2{margin:0 0 10px;font-size:17px}
 .row{margin:11px 0}
 label{display:block;font-size:13px;color:#9aa0aa;margin-bottom:5px}
 .val{float:right;color:#e6e6e6;font-variant-numeric:tabular-nums}
 input[type=range]{width:100%;accent-color:#2d6cdf}
 input[type=number]{background:#22262f;color:#e6e6e6;border:1px solid #2a2e37;border-radius:6px;padding:6px;width:80px}
 select{background:#22262f;color:#e6e6e6;border:1px solid #2a2e37;border-radius:8px;padding:8px;width:100%}
 .toggle{display:flex;gap:8px}.toggle button{flex:1}
 .on{background:#2d6cdf;border-color:#2d6cdf;color:#fff}
 .enbtn.on{background:#1f8a4c;border-color:#1f8a4c;color:#fff}
 .btns{display:flex;gap:8px;margin-top:8px}
 .run{background:#1f8a4c;border-color:#1f8a4c;color:#fff}
 .stop{background:#b23b3b;border-color:#b23b3b;color:#fff}
 .status{padding:2px 18px 16px;color:#9aa0aa;font-size:13px}
 .estop{width:100%;background:#b23b3b;border-color:#b23b3b;color:#fff;font-size:18px;font-weight:700;padding:14px;letter-spacing:.5px;cursor:pointer}
 h3{font-size:12px;color:#9aa0aa;text-transform:uppercase;letter-spacing:.5px;margin:14px 0 6px;border-top:1px solid #2a2e37;padding-top:10px}
 #settings{padding:14px;display:none}
 .sgrid{display:grid;grid-template-columns:auto 90px 90px;gap:8px 12px;align-items:center;max-width:380px}
 .sgrid .h{color:#9aa0aa;font-size:12px}
 .axlist{display:flex;flex-wrap:wrap;gap:12px;margin:10px 0}
 .axlist label{display:flex;align-items:center;gap:6px;color:#e6e6e6}
 *{box-sizing:border-box}button{min-height:44px}button:disabled{opacity:.45;cursor:not-allowed}button:focus-visible,input:focus-visible,select:focus-visible,summary:focus-visible{outline:2px solid #93b5ff;outline-offset:3px}header{position:sticky;top:0;z-index:10;gap:12px}header b{font-size:17px}.topstop{background:#b23b3b;color:#fff;font-weight:700;min-width:130px}.page{max-width:1140px;margin:auto;padding:24px 20px 40px}.controller{border:1px solid #2a2e37;border-radius:12px;background:#171a21;padding:24px}.controller.live{border-color:#3fb765}.gpheading{display:flex;align-items:center;justify-content:space-between;gap:18px}.eyebrow{font-size:11px;letter-spacing:1.8px;color:#9aa0aa;font-weight:600}h1{font-size:30px;margin:6px 0 10px;font-weight:650;letter-spacing:-.7px}.muted,.gpnote{color:#aeb4bf;font-size:13px;line-height:1.6}.statepill{padding:8px 12px;border:1px solid #626977;border-radius:6px;color:#c3c9d3;font-size:13px;white-space:nowrap}.statepill.enabled{color:#84dea7;border-color:#3fb765}.gpbar{display:flex;align-items:center;gap:18px;margin:22px 0 26px}.primary{background:#2d6cdf;color:white;font-weight:650;min-width:200px}.controllergrid{display:grid;grid-template-columns:1.4fr 1fr;gap:36px}.controlmap{display:grid;grid-template-columns:1fr 1fr;gap:22px;background:#101319;border:1px solid #2a2e37;padding:22px;border-radius:8px}.stickunit{display:flex;align-items:center;flex-direction:column;gap:6px;min-width:0}.stick{position:relative;border:1px solid #626977;background:#171a21;width:110px;height:110px;border-radius:50%;display:grid;place-items:center;margin:3px 0 10px}.stick span{color:#626977;font-size:28px}.stick i{position:absolute;left:calc(50% - 9px);top:calc(50% - 9px);height:18px;width:18px;background:#93b5ff;border-radius:50%;z-index:1}.stickunit output,.triggers output{font-size:12px;color:#aeb4bf;font-variant-numeric:tabular-nums}.triggers{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr;gap:8px 18px}.triggers label{margin:0}.triggers output{grid-column:1/-1;text-align:center}meter{display:block;width:100%;height:14px;margin-top:5px;accent-color:#2d6cdf}.quickcontrols{padding:4px 0}.quickcontrols label{font-size:14px;color:#e6e6e6}.quickcontrols input[type=range]{margin:10px 0 0}.quickcontrols label output{float:right;font-variant-numeric:tabular-nums}.quickcontrols .toggle button{font-size:18px;font-weight:650}.quickcontrols p{margin:8px 0 20px}kbd{font-family:inherit;border:1px solid #626977;border-radius:5px;padding:2px 7px;font-size:12px;font-weight:650;color:#e6e6e6}.buttonlegend{display:grid;grid-template-columns:1fr 1fr;gap:12px;font-size:13px}.buttonlegend>span{line-height:1.8;padding:5px 0}.buttonlegend .pressed{color:#93b5ff}.gpnote{border-top:1px solid #2a2e37;padding-top:18px;margin:24px 0 18px}details{border-top:1px solid #2a2e37;margin-top:16px;padding-top:16px}summary{cursor:pointer;min-height:44px;line-height:1.6;color:#e6e6e6;font-size:14px}.tunegrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.tunegrid input[type=number]{display:block;width:100%;margin-top:5px}.tunegrid select{margin-top:5px}.mappingrow{display:flex;align-items:center;gap:12px;margin:14px 0;flex-wrap:wrap}.mappingrow label{flex:1}.mappingrow input[type=number]{display:block;width:100%}.actionrow{display:flex;align-items:center;justify-content:space-between;gap:12px;margin:10px 0;font-size:13px}.manual{margin-top:24px}.manual fieldset{border:0;padding:0;margin:0;min-width:0}.manual .card{min-width:270px}.manual .wrap{padding:14px 0}.manual #settings{padding:14px 0}.status{max-width:1140px;margin:auto;padding:0 20px 24px;min-height:40px}#gpdevice{max-width:620px;overflow-wrap:anywhere}#gplink{font-variant-numeric:tabular-nums}#resetresponse{margin-top:18px}.manualintro{color:#aeb4bf;font-size:13px;line-height:1.7}
 @media(max-width:720px){header{padding:10px 14px}.page{padding:16px 12px 30px}.controller{padding:18px 14px}.gpheading{align-items:flex-start;flex-direction:column;gap:12px}h1{font-size:26px}.controllergrid{grid-template-columns:1fr;gap:24px}.gpbar{align-items:flex-start;flex-direction:column;gap:10px}.primary{width:100%}.controlmap{padding:16px;gap:18px}.tunegrid{grid-template-columns:1fr 1fr}.statepill{font-size:12px}.actionrow{align-items:flex-start;flex-wrap:wrap}.manual .card{min-width:0;flex-basis:100%}.stick{width:100px;height:100px}.stickunit .muted{font-size:12px}.topstop{min-width:110px}.status{padding:0 12px 20px}}@media(prefers-reduced-motion:reduce){html{scroll-behavior:auto}}

</style></head><body>
<header><b>PNP · Controller</b><button id="estop" class="topstop">■ STOP <small> / B</small></button></header>
<main class="page"><div id="gppanel"></div>
<details class="manual" id="manualpanel"><summary>Manual motor controls & machine settings</summary>
<fieldset id="manualcontrols"><p class="manualintro">Disable the controller before manual moves. Set motor values, enable its driver, then choose Run turns or Spin. Turn counts use the configured steps per revolution.</p>
<button onclick="toggleSettings()">Machine settings</button><div id="settings"></div>
<div class="wrap" id="vacwrap"></div><div class="wrap" id="wrap"></div></fieldset></details></main>
<div class="status" id="status" role="status" aria-live="polite">Connecting to machine…</div>
<script>
const MS=[1,2,4,8,16,32,64,128,256];
const MOTOR={C0:'Nema 8 · 0.6A',C1:'Nema 8 · 0.6A',X:'Nema 17 · 2.1A',Y0:'Nema 17 · 2.1A',Y1:'Nema 17 · 2.1A',Z:'Nema 17 · 2.1A'};
const status=document.getElementById('status');
let S={}, AX=[], VAC={l:0,r:0}, YLINK=0,ZLIMIT={turns:0,positiveUp:1,topKnown:0};
function api(u){return http(u).then(t=>{status.textContent=t;return t;}).catch(e=>{status.textContent=e.message;});}
function act(on,off){on.classList.add('on');off.classList.remove('on');}

function renderVac(){
 document.getElementById('vacwrap').innerHTML=`
  <div class="card"><h2>Vacuum</h2>
   <div class="row"><label>Vacuum Left (GPIO4)</label>
    <div class="toggle"><button id="vl0">OFF</button><button id="vl1">ON</button></div></div>
   <div class="row"><label>Vacuum Right (GPIO7)</label>
    <div class="toggle"><button id="vr0">OFF</button><button id="vr1">ON</button></div></div>
  </div>`;
 const vl0=gid('vl0'),vl1=gid('vl1'),vr0=gid('vr0'),vr1=gid('vr1');
 act(VAC.l?vl1:vl0, VAC.l?vl0:vl1); act(VAC.r?vr1:vr0, VAC.r?vr0:vr1);
 vl0.onclick=()=>{api('/vac?side=left&on=0');act(vl0,vl1);};
 vl1.onclick=()=>{api('/vac?side=left&on=1');act(vl1,vl0);};
 vr0.onclick=()=>{api('/vac?side=right&on=0');act(vr0,vr1);};
 vr1.onclick=()=>{api('/vac?side=right&on=1');act(vr1,vr0);};
}

function render(){
 renderVac();
 // ---- settings panel ----
 const rng=(k,lbl)=>`<div class="h">${lbl}</div>
   <input type="number" id="${k}Min" value="${S[k+'Min']}"><input type="number" id="${k}Max" value="${S[k+'Max']}">`;
 document.getElementById('settings').innerHTML=`
  <h2 style="margin:4px 0">Slider ranges</h2>
  <div class="sgrid">
   <div class="h"></div><div class="h">min</div><div class="h">max</div>
   ${rng('cur','Current (mA)')}${rng('spd','Speed (Hz)')}${rng('acc','Accel (st/s²)')}${rng('turn','Turns')}
   <div class="h">Steps/rev</div><input type="number" id="spr" value="${S.spr}"><div></div>
  </div>
  <h2 style="margin:14px 0 4px">Active axes</h2>
  <div class="axlist">${AX.map(a=>`<label><input type="checkbox" ${a.active?'checked':''} id="ax${a.n}">${a.n}${a.fs?'':' (no step ch!)'}</label>`).join('')}</div>
  <h2 style="margin:14px 0 4px">Y pairing</h2>
  <label class="axlist"><input type="checkbox" id="ylink" ${YLINK?'checked':''}> Link Y0 ↔ Y1 (spin / move together)</label>
  <button class="run" onclick="saveSettings()">Save settings</button>`;
 gid('ylink').onchange=e=>api('/pair?on='+(e.target.checked?1:0));
 // ---- axis cards ----
 document.getElementById('wrap').innerHTML=AX.filter(a=>a.active).map(card).join('')||'<div class="status">No active axes — enable some in ⚙ Settings.</div>';
 AX.filter(a=>a.active).forEach(wire);
 status.textContent='UART conn '+AX.map(a=>a.n+'='+a.conn).join(', ')+'  (0=OK)';
}
function card(a){const i=a.n;return `
 <div class="card"><h2>${a.n} <span style="font-size:12px;color:#9aa0aa;font-weight:400">${MOTOR[a.n]||''}</span> <span class="val" style="font-size:12px;color:${a.conn==0?'#3fb765':'#e06b6b'}">${a.conn==0?'● online':'● no UART'}</span></h2>
  <div class="row"><label>Run current <span class="val" id="cv${i}"></span> mA (max ${a.max})</label>
   <input type="range" min="0" max="${a.max}" step="5" id="c${i}"></div>
  <div class="row"><label>Speed <span class="val" id="sv${i}"></span> Hz</label>
   <input type="range" min="${S.spdMin}" max="${S.spdMax}" step="10" id="s${i}"></div>
  <div class="row"><label>Acceleration <span class="val" id="av${i}"></span> st/s²</label>
   <input type="range" min="${S.accMin}" max="${S.accMax}" step="100" id="a${i}"></div>
  <div class="row"><label>Microsteps</label><select id="m${i}">${MS.map(x=>`<option>${x}</option>`).join('')}</select></div>
  <h3>Mode</h3>
  <div class="row"><label>Direction</label><div class="toggle"><button id="d${i}0">CW</button><button id="d${i}1">CCW</button></div></div>
  <div class="row"><label>Chopper (spread = more torque)</label><div class="toggle"><button id="p${i}0">stealth</button><button id="p${i}1">spread</button></div></div>
  <div class="row"><label>Driver — must be ON to move</label><div class="toggle"><button id="e${i}0">OFF</button><button id="e${i}1" class="enbtn">ON</button></div></div>
  <h3>Run</h3>
  <div class="row"><label>Move a set number of turns</label>
   <input type="number" id="t${i}" min="${S.turnMin}" max="${S.turnMax}" step="0.25" value="1"> turns
   <div class="btns"><button class="run" id="rt${i}">▶ Run turns</button>
    <button class="run" id="sp${i}">∞ Spin</button><button class="stop" id="st${i}">■ Stop</button></div></div>
 </div>`;}
function wire(a){const i=a.n;
 const c=gid('c'+i),cv=gid('cv'+i);c.value=a.cur;cv.textContent=a.cur;
 c.oninput=()=>{cv.textContent=c.value;deb(c,()=>api(`/set?axis=${i}&field=current&value=${c.value}`));};
 const s=gid('s'+i),sv=gid('sv'+i);s.value=a.spd;sv.textContent=a.spd;
 s.oninput=()=>{sv.textContent=s.value;deb(s,()=>api(`/set?axis=${i}&field=speed&value=${s.value}`));};
 const ac=gid('a'+i),av=gid('av'+i);ac.value=a.acc;av.textContent=a.acc;
 ac.oninput=()=>{av.textContent=ac.value;deb(ac,()=>api(`/set?axis=${i}&field=accel&value=${ac.value}`));};
 gid('m'+i).value=a.mic;gid('m'+i).onchange=e=>api(`/set?axis=${i}&field=micro&value=${e.target.value}`);
 bind(i,'d','dir',a.dir);bind(i,'p','spread',a.spread);bind(i,'e','enabled',a.en);
 gid('rt'+i).onclick=()=>api(`/run?axis=${i}&turns=${gid('t'+i).value}`);
 gid('sp'+i).onclick=()=>api(`/run?axis=${i}&mode=spin`);
 gid('st'+i).onclick=()=>api(`/stop?axis=${i}`);
}
function bind(i,p,f,v){const b0=gid(p+i+'0'),b1=gid(p+i+'1');
 act(v?b1:b0, v?b0:b1);
 b0.onclick=()=>{api(`/set?axis=${i}&field=${f}&value=0`);act(b0,b1);};
 b1.onclick=()=>{api(`/set?axis=${i}&field=${f}&value=1`);act(b1,b0);};}
function gid(x){return document.getElementById(x);}
function deb(el,fn){clearTimeout(el._t);el._t=setTimeout(fn,120);}
function toggleSettings(){gid('manualpanel').open=true;const e=gid('settings');e.style.display=e.style.display=='none'||!e.style.display?'block':'none';}
function saveSettings(){
 const q=new URLSearchParams();
 ['cur','spd','acc','turn'].forEach(k=>{q.set(k+'Min',gid(k+'Min').value);q.set(k+'Max',gid(k+'Max').value);});
 q.set('spr',gid('spr').value);
 AX.forEach(a=>q.set('ax'+a.n,gid('ax'+a.n).checked?1:0));
 api('/settings?'+q.toString()).then(()=>load());
}
async function load(){try{const d=JSON.parse(await http('/state'));S=d.set;AX=d.ax;VAC=d.vac;YLINK=d.ylink;ZLIMIT=d.zlimit||ZLIMIT;render();renderGp();}catch(e){status.textContent=e.message;}}

// ---------- Xbox / Gamepad control ----------
function stored(key,fallback){try{return JSON.parse(localStorage.getItem(key))||fallback;}catch{return fallback;}}
const SCH_DEF={xy:{fast:12000,slow:120,dz:0.16,micro:1,accel:30000,invX:true,invY:true},z:{fast:12000,slow:100,dz:0.10,micro:1,accel:20000,invert:false},c:{fast:12000,slow:100,dz:0.16,micro:1,accel:20000,invert:false}};
let scheme=stored('pnpScheme',{});
for(const k of ['xy','z','c']){scheme[k]=Object.assign({},SCH_DEF[k],scheme[k]);scheme[k].micro=1;}
// Apply the requested defaults once to existing browsers as well as new profiles.
// Later user tuning remains editable and is preserved across reloads.
if(localStorage.getItem('pnpJogDefaults')!=='2026-10-24-12k'){
 for(const k of ['xy','z','c'])scheme[k].fast=12000;
 scheme.xy.invX=true;
 localStorage.setItem('pnpScheme',JSON.stringify(scheme));
 localStorage.setItem('pnpJogDefaults','2026-10-24-12k');
}
let gpMap=stored('pnpGpMap',{});gpMap.ax=gpMap.ax||{};gpMap.btn=Object.assign({stopAll:'b1'},gpMap.btn||{});gpMap.btn.vacL='b4';gpMap.btn.vacR='b5';
let vacuumPending={l:false,r:false};
let macros=stored('pnpMacros',{}),GP=null,learn=null,cActive='C0 + C1',armed=false,arming=false,epoch=0;
let mode=localStorage.getItem('pnpControlMode')||'standard',sensitivity=Math.max(10,Math.min(100,+(localStorage.getItem('pnpSensitivity')||100)));
let socket=null,sequence=0,pendingSequence=0,pendingAt=0;
let prevB=[],pendingFrame=false,lastSent=-Infinity,lastKey='',lastAck=0,rtt=0,lastPaint=0,armGeneration=0;
const ACTIONS=[['stopAll','STOP ALL'],['m1go','Recall position 1'],['m2go','Recall position 2'],['m3go','Recall position 3'],['zero','Zero positions']];
const ZERO=()=>({x:0,y:0,z:0,c0:0,c1:0,precision:0});
const esc=v=>String(v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function saveSch(){localStorage.setItem('pnpScheme',JSON.stringify(scheme));}
function saveGp(){localStorage.setItem('pnpGpMap',JSON.stringify(gpMap));}
function saveMac(){localStorage.setItem('pnpMacros',JSON.stringify(macros));}
function getPad(){try{const pads=navigator.getGamepads?navigator.getGamepads():[];if(GP!==null&&pads[GP]?.connected)return pads[GP];return null;}catch{return null;}}
function discoverPad(){try{const pads=Array.from(navigator.getGamepads?navigator.getGamepads():[]);const gp=pads.find(p=>p&&p.connected);GP=gp?gp.index:null;}catch{GP=null;}}
async function http(url,method='GET',timeout=3000){const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),timeout);try{const r=await fetch(url,{method,cache:'no-store',signal:abort.signal});const t=await r.text();if(!r.ok)throw Error(t||'Controller request failed');return t;}finally{clearTimeout(timer);}}
function setMessage(t){status.textContent=t;}
function paintState(message){const gp=getPad(),state=gid('gpstate'),arm=gid('gparm');if(state){state.textContent=message||(armed?'Enabled · ready to move':gp?'Connected · controller disabled':'No controller');state.classList.toggle('enabled',armed);}if(arm){arm.textContent=arming?'Preparing motors…':armed?'Disable controller':'Enable controller';arm.disabled=arming||!gp||!!learn||!socket||socket.readyState!==WebSocket.OPEN;}const panel=gid('controller');if(panel)panel.classList.toggle('live',armed);const field=gid('manualcontrols');if(field)field.disabled=armed;for(const id of ['padvacL','padvacR']){const e=gid(id);if(e)e.disabled=!armed;}gid('gpadvanced')?.querySelectorAll('input,select,button').forEach(e=>{e.disabled=armed||arming||(e.classList.contains('gpr')&&!macros[e.dataset.s]);});}
async function halt(reason='Stopped',disable=false){++armGeneration;armed=false;arming=false;pendingFrame=false;pendingSequence=0;lastKey='';paintState(reason);setMessage(reason);try{await http(disable?'/controller?action=stop':'/stopall',disable?'POST':'GET',1200);}catch{setMessage(reason+' · connection unavailable; machine timeout will stop controller motion');} }
async function enableController(){if(!socket||socket.readyState!==WebSocket.OPEN){setMessage('Controller connection is not ready.');return;}if(armed){await halt('Controller disabled',true);await load();return;}const gp=getPad();if(!gp)return;if(Object.entries(desiredFrame(gp)).some(([k,v])=>k!=='precision'&&v!==0)){setMessage('Center both sticks and release the triggers before enabling.');return;}
 const generation=++armGeneration;arming=true;paintState();const q=new URLSearchParams({action:'arm'});for(const [g,m,a] of [['xy','mxy','axy'],['z','mz','az'],['c','mc','ac']]){q.set(m,scheme[g].micro);q.set(a,scheme[g].accel);}
 try{const data=JSON.parse(await http('/controller?'+q,'POST',5000));if(generation!==armGeneration||document.hidden||!getPad()){await http('/controller?action=stop','POST');return;}epoch=data.epoch;armed=true;lastSent=-Infinity;lastAck=performance.now();lastKey='';setMessage('Controller enabled. B or STOP halts motion. Hold X for precision.');paintState();sendFrame(ZERO(),performance.now());}
 catch(e){setMessage(e.message);armed=false;}finally{if(generation===armGeneration){arming=false;paintState();}} }
function qhz(v,cfg,precision=false){const a=Math.min(1,Math.abs(v)),dz=Math.max(.02,Math.min(.5,+cfg.dz||.16));if(a<=dz)return 0;const t=(a-dz)/(1-dz),cap=Math.min(+S.spdMax||20000,Math.max(50,+cfg.fast||1000));const slow=Math.min(cap,Math.max(1,+cfg.slow||100));// Tuning rates are kept in 1/16-step equivalent units to retain existing travel speeds.
 const micro=precision?16:1;const hz=Math.round((slow+t*(cap-slow))*(sensitivity/100)*(precision?.25:1)*micro/16);return Math.sign(v)*Math.max(1,hz);}
function desiredFrame(gp){const f=ZERO(),precision=!!gp.buttons[2]?.pressed;f.precision=precision?1:0;if(mode==='custom'){for(const [i,m] of Object.entries(gpMap.ax)){if(!m.dest)continue;let v=gp.axes[i]||0;if(m.inv)v=-v;const key={X:'x',Y0:'y',Y1:'y',Z:'z',C0:'c0',C1:'c1'}[m.dest];if(!key)continue;const n=qhz(v,{dz:m.dz??.15,fast:m.scale||2000,slow:100},precision);if(Math.abs(n)>Math.abs(f[key]))f[key]=n;}f.c0=f.c1=(Math.abs(f.c0)>=Math.abs(f.c1)?f.c0:f.c1);return f;}
 f.x=qhz((gp.axes[0]||0)*(scheme.xy.invX?-1:1),scheme.xy,precision);f.y=qhz((gp.axes[1]||0)*(scheme.xy.invY?-1:1),scheme.xy,precision);
 f.z=qhz(((gp.buttons[7]?.value||0)-(gp.buttons[6]?.value||0))*(scheme.z.invert?-1:1),scheme.z,precision);
 f.c0=f.c1=qhz((gp.axes[2]||0)*(scheme.c.invert?-1:1),scheme.c,precision);return f;}
function connectControlSocket(){
 socket=new WebSocket('ws://'+location.hostname+':81/');
 socket.onopen=()=>{paintState();if(!armed)setMessage('Controller link connected. Center the sticks, then enable.');};
 socket.onmessage=e=>{try{const d=JSON.parse(e.data);if(d.error){pendingFrame=false;if(armed)halt(d.error);return;}if(d.seq===pendingSequence&&pendingFrame){pendingFrame=false;if(armed){rtt=performance.now()-pendingAt;lastAck=performance.now();}}}catch{if(armed)halt('Controller paused · invalid machine response');}};
 socket.onclose=()=>{pendingFrame=false;if(armed||arming)halt('Controller paused · connection lost');paintState();setTimeout(connectControlSocket,1500);};
 socket.onerror=()=>{if(!armed)setMessage('Controller link unavailable · reconnecting…');};
}
function sendFrame(frame,now){if(!armed||pendingFrame||!socket||socket.readyState!==WebSocket.OPEN)return;const key=JSON.stringify(frame);if(now-lastSent<40||(key===lastKey&&now-lastSent<100))return;
 pendingFrame=true;lastSent=now;lastKey=key;pendingAt=now;sequence=sequence%1000000000+1;pendingSequence=sequence;
 try{socket.send([sequence,epoch,frame.x,frame.y,frame.z,frame.c0,frame.c1,frame.precision].join(','));}catch{pendingFrame=false;halt('Controller paused · connection lost');}
}
function tuningGroup(g,label,extra){const c=scheme[g];return `<h3>${label}</h3><div class="tunegrid"><label>Top speed (1/16-step Hz)<input type="number" class="sc" data-g="${g}" data-k="fast" min="50" max="${S.spdMax||20000}" value="${c.fast}"></label><label>Start speed (1/16-step Hz)<input type="number" class="sc" data-g="${g}" data-k="slow" min="20" value="${c.slow}"></label><label>Deadzone<input type="number" class="sc" data-g="${g}" data-k="dz" min="0.02" max="0.5" step="0.02" value="${c.dz}"></label><label>Resolution<p>Normal: 1<br>Hold X: 1/16</p></label><label>Accel (1/16-step units)<input type="number" class="sc" data-g="${g}" data-k="accel" min="100" max="${S.accMax||50000}" value="${c.accel}"></label>${extra}</div>`;}
function renderGp(){const el=gid('gppanel');if(!el)return;const wasOpen=gid('gpadvanced')?.open, gp=getPad();
 el.innerHTML=`<section class="controller" id="controller"><div class="gpheading"><div><div class="eyebrow">XBOX CONTROL</div><h1>Move the machine.</h1><div id="gpdevice" class="muted"></div></div><span class="statepill" id="gpstate"></span></div>
 <div class="gpbar"><button class="primary" id="gparm">Enable controller</button><span class="muted" id="gplink">Waiting for controller</span></div>
 <div class="controllergrid"><div class="controlmap"><div class="stickunit"><div class="stick" aria-label="Left stick input"><i id="leftdot"></i><span>+</span></div><b>Left stick</b><span class="muted">X / paired Y motors</span><output id="xyread">X 0 · Y 0 Hz</output></div><div class="stickunit"><div class="stick" aria-label="Right stick input"><i id="rightdot"></i><span>+</span></div><b>Right stick</b><span class="muted">Rotate <b id="cact">${cActive}</b></span><output id="cread">0 Hz</output></div><div class="triggers"><label>LT · Z down<meter id="ltmeter" min="0" max="1" value="0"></meter></label><label>RT · Z up<meter id="rtmeter" min="0" max="1" value="0"></meter></label><output id="zread">Z 0 Hz</output><output id="zsavedstatus" style="grid-column:1/-1">${zLimitStatus()}</output></div></div>
 <div class="quickcontrols"><label>Right stick motors</label><p><b>C0 + C1 · always together</b></p><label for="sensitivity">Jog speed <output id="sensitivityvalue">${sensitivity}%</output></label><input id="sensitivity" type="range" min="10" max="100" step="5" value="${sensitivity}"><p class="muted">Normal: full steps. Hold <kbd>X</kbd> for 1/16-step precision at quarter speed. <b id="stepmode">Full step · 1</b></p><div class="buttonlegend"><span id="chipB"><kbd>B</kbd> Stop</span><span id="chipLB"><kbd>LB</kbd> Left vacuum</span><span id="chipRB"><kbd>RB</kbd> Right vacuum</span><span id="chipX"><kbd>X</kbd> Precision</span></div><div class="toggle"><button id="padvacL">LB · Left vacuum</button><button id="padvacR">RB · Right vacuum</button></div><p class="muted" id="vacread">Vacuums · left ${VAC.l?'ON':'OFF'} · right ${VAC.r?'ON':'OFF'}</p></div></div>
 <p class="gpnote">Enable prepares all six motors. Center the sticks first. Disconnecting the controller, leaving this page, or losing updates stops controller motion. Motor current is unchanged.</p>
 <details id="gpadvanced" ${wasOpen?'open':''}><summary>Controller tuning & saved positions</summary><p class="muted">Disable the controller to change these settings.</p><h3>Z travel limits · zero at top</h3><p><b id="zlimitapplied">${zLimitStatus()}</b></p><p id="zlimitdraft" class="muted" role="status"></p><p class="muted">${zLimitStatus()}<br>Travel is in motor revolutions, not millimetres. Set 0 for no lower limit. Re-establish top zero after each restart or any manual movement while unpowered. Clear the reference before jogging to an unknown top.</p><div class="tunegrid"><label>Travel below top (motor turns)<input id="ztravel" type="number" min="0" step="0.01" value="${ZLIMIT.turns}"></label><label><input id="zpositive" type="checkbox" ${ZLIMIT.positiveUp?'checked':''}> Positive motor direction is up</label></div><div class="btns"><button id="savezlimits">Save Z limits</button><button id="setztop">Set current Z as top (0)</button><button id="clearztop">Clear top reference</button></div><p class="muted">Set top zero to immediately protect the top. Leave travel at 0 for no lower limit, or save a measured distance to protect the bottom too. Confirm the up direction first. Software limits cannot detect missed steps or a moved motor.</p><label>Control layout<select id="controlmode"><option value="standard">Xbox preset</option><option value="custom">Custom stick mappings</option></select></label>
 ${tuningGroup('xy','Left stick · X / Y',`<label><input class="sc" type="checkbox" data-g="xy" data-k="invX" ${scheme.xy.invX?'checked':''}> Reverse X</label><label><input class="sc" type="checkbox" data-g="xy" data-k="invY" ${scheme.xy.invY?'checked':''}> Reverse Y</label>`)}${tuningGroup('z','Triggers · Z',`<label><input class="sc" type="checkbox" data-g="z" data-k="invert" ${scheme.z.invert?'checked':''}> Reverse Z</label>`)}${tuningGroup('c','Right stick · C0 / C1',`<label><input class="sc" type="checkbox" data-g="c" data-k="invert" ${scheme.c.invert?'checked':''}> Reverse rotation</label>`)}
 <details><summary>Custom mappings & button learning</summary><p class="muted">B always stops. LB/RB always toggle left/right vacuum. X is precision. C0 and C1 always run together.</p>${[0,1,2,3].map(i=>{const m=gpMap.ax[i]||{};return `<div class="mappingrow"><label>Stick axis ${i}<select class="gpd" data-i="${i}"><option value="">Unassigned</option>${['X','Y0','Y1','Z','C0','C1'].map(n=>`<option ${m.dest===n?'selected':''}>${n}</option>`).join('')}</select></label><label>Max Hz<input class="gps" data-i="${i}" type="number" min="50" value="${+m.scale||2000}"></label><label>Deadzone<input class="gpz" data-i="${i}" type="number" min="0.02" max="0.5" step="0.02" value="${+m.dz||.15}"></label><label><input class="gpi" data-i="${i}" type="checkbox" ${m.inv?'checked':''}> Reverse</label></div>`;}).join('')}${ACTIONS.map(([a,label])=>`<div class="actionrow"><span>${label} · <b>${esc(gpMap.btn[a]||'unassigned')}</b></span><button class="gpl" data-a="${a}">${learn===a?'Press a button…':'Learn'}</button></div>`).join('')}</details>
 <h3>Saved positions</h3>${[1,2,3].map(n=>`<div class="actionrow"><span>Position ${n} · ${macros[n]?'saved':'empty'}</span><div><button class="gpm" data-s="${n}">Save current</button> <button class="gpr" data-s="${n}" ${macros[n]?'':'disabled'}>Recall</button></div></div>`).join('')}<button id="resetresponse">Restore responsive defaults</button></details></section>`;
 gid('gpdevice').textContent=gp?gp.id:(!navigator.getGamepads?'Gamepad access unavailable in this browser.':'Connect an Xbox controller, then press any button.');
 gid('gparm').onclick=enableController;gid('padvacL').onclick=()=>doAction('vacL');gid('padvacR').onclick=()=>doAction('vacR');
 gid('sensitivity').oninput=e=>{sensitivity=+e.target.value;gid('sensitivityvalue').textContent=sensitivity+'%';localStorage.setItem('pnpSensitivity',sensitivity);};
 gid('controlmode').value=mode;gid('controlmode').onchange=e=>{mode=e.target.value;localStorage.setItem('pnpControlMode',mode);};
 el.querySelectorAll('.sc').forEach(x=>x.onchange=e=>{const g=e.target.dataset.g,k=e.target.dataset.k;let v=e.target.type==='checkbox'?e.target.checked:+e.target.value;if(k==='dz')v=Math.max(.02,Math.min(.5,v));if(k==='accel')v=Math.max(100,Math.min(S.accMax||50000,v));if(k==='fast'||k==='slow')v=Math.max(20,Math.min(S.spdMax||20000,v));scheme[g][k]=v;saveSch();});
 for(const [selector,key,convert] of [['.gpd','dest',v=>v],['.gps','scale',Number],['.gpz','dz',Number],['.gpi','inv',Boolean]])el.querySelectorAll(selector).forEach(x=>x.onchange=e=>{const i=e.target.dataset.i;gpMap.ax[i]=gpMap.ax[i]||{};gpMap.ax[i][key]=convert(e.target.type==='checkbox'?e.target.checked:e.target.value);saveGp();});
 el.querySelectorAll('.gpl').forEach(x=>x.onclick=()=>{learn=learn===x.dataset.a?null:x.dataset.a;renderGp();});el.querySelectorAll('.gpm').forEach(x=>x.onclick=()=>saveMacro(x.dataset.s));el.querySelectorAll('.gpr').forEach(x=>x.onclick=()=>gotoMacro(x.dataset.s));
 gid('savezlimits').onclick=()=>updateZLimits(false);gid('setztop').onclick=()=>updateZLimits(true);gid('clearztop').onclick=async()=>{try{const text=await http('/zlimits?action=forget','POST');await load();setMessage(text);}catch(e){setMessage(e.message);}};gid('ztravel').oninput=zLimitDraft;gid('zpositive').onchange=zLimitDraft;
 gid('resetresponse').onclick=()=>{scheme=JSON.parse(JSON.stringify(SCH_DEF));saveSch();renderGp();};paintState();}
function toggleGp(){gid('gppanel').scrollIntoView({behavior:'smooth'});}
function zLimitStatus(){return ZLIMIT.topKnown?(ZLIMIT.turns>0?'Z limits ON · top 0 / bottom '+ZLIMIT.turns+' motor turns below':'Z TOP LIMIT ON · zero at top; no lower limit'):(ZLIMIT.turns>0?'Z BLOCKED · '+ZLIMIT.turns+' turns saved; set top zero':'Z limits OFF · top zero not set');}
function zLimitDraft(){const travel=gid('ztravel'),up=gid('zpositive'),note=gid('zlimitdraft');if(!travel||!up||!note)return;const dirty=Number(travel.value)!==Number(ZLIMIT.turns)||up.checked!==!!ZLIMIT.positiveUp;note.textContent=dirty?'Unsaved limit changes — click Save Z limits to apply.':'';}
async function updateZLimits(top){
 const draft={turns:gid('ztravel').value,positiveUp:gid('zpositive').checked};
 const q=top?new URLSearchParams({action:'top',positiveUp:draft.positiveUp?1:0}):new URLSearchParams({action:'save',turns:draft.turns,positiveUp:draft.positiveUp?1:0});
 const buttons=[gid('savezlimits'),gid('setztop')];buttons.forEach(b=>b.disabled=true);
 try{
  const text=await http('/zlimits?'+q,'POST');
  const d=JSON.parse(await http('/state'));S=d.set;AX=d.ax;VAC=d.vac;YLINK=d.ylink;ZLIMIT=d.zlimit;
  render();renderGp();
  if(top){gid('ztravel').value=draft.turns;gid('zpositive').checked=draft.positiveUp;}
  zLimitDraft();setMessage(text+(top&&Number(draft.turns)!==Number(ZLIMIT.turns)?' · Travel value preserved; click Save Z limits to apply it.':''));
 }catch(e){setMessage(e.message);zLimitDraft();}
 finally{paintState();}
}
async function saveMacro(n){try{const d=JSON.parse(await http('/state'));const m={};for(const a of d.ax)if(a.active)m[a.n]=Math.round(a.pos*16/a.mic);macros[n]=m;saveMac();renderGp();setMessage('Saved position '+n);}catch(e){setMessage(e.message);}}
async function gotoMacro(n){const m=macros[n];if(!m)return;await halt('Moving to saved position '+n);const d=JSON.parse(await http('/state'));const micros=Object.fromEntries(d.ax.map(a=>[a.n,a.mic]));for(const [axis,pos] of Object.entries(m)){if(YLINK&&axis==='Y1')continue;await api(`/goto?axis=${axis}&pos=${Math.round(pos*(micros[axis]||1)/16)}`);} }
async function doAction(a){if(a==='stopAll')return halt('Stopped · enable controller to resume');if(a==='vacL'||a==='vacR'){if(!armed)return;const side=a==='vacL'?'l':'r';if(vacuumPending[side])return;vacuumPending[side]=true;const next=VAC[side]?0:1;try{await http('/vac?side='+(side==='l'?'left':'right')+'&on='+next);VAC[side]=next;gid('vacread').textContent=`Vacuums · left ${VAC.l?'ON':'OFF'} · right ${VAC.r?'ON':'OFF'}`;}catch(e){setMessage(e.message);}finally{vacuumPending[side]=false;} }else if(a==='zero'){await halt('Controller stopped');await api('/zero?axis=all');}else if(a[0]==='m')await gotoMacro(a[1]);}
function paintInputs(gp,f,now){if(now-lastPaint<33)return;lastPaint=now;const lx=gp?.axes[0]||0,ly=gp?.axes[1]||0,rx=gp?.axes[2]||0,ry=gp?.axes[3]||0;
 gid('stepmode').textContent=f.precision?'Precision · 1/16':'Full step · 1';
 gid('leftdot').style.transform=`translate(${lx*32}px,${ly*32}px)`;gid('rightdot').style.transform=`translate(${rx*32}px,${ry*32}px)`;
 gid('ltmeter').value=gp?.buttons[6]?.value||0;gid('rtmeter').value=gp?.buttons[7]?.value||0;gid('xyread').textContent=`X ${f.x} · Y ${f.y} Hz`;gid('cread').textContent=`${f.c0} Hz`;gid('zread').textContent=`Z ${f.z} Hz`;
 for(const [id,i] of [['chipB',1],['chipLB',4],['chipRB',5],['chipX',2]])gid(id)?.classList.toggle('pressed',!!gp?.buttons[i]?.pressed);
 gid('gplink').textContent=armed?(rtt?`${Math.round(rtt)} ms · live`:'Connecting…'):gp?'Inputs visible · motion disabled':'Waiting for controller';}
function poll(now){requestAnimationFrame(poll);let gp=getPad();if(!gp&&GP!==null){GP=null;if(armed||arming)halt('Controller disconnected');renderGp();}if(!gp&&GP===null){discoverPad();gp=getPad();if(gp)renderGp();}if(!gid('leftdot'))return;
 const f=gp?desiredFrame(gp):ZERO();paintInputs(gp,f,now);if(!gp)return;
 const b=gp.buttons.map(v=>v.pressed||v.value>.5);
 if(learn){const i=b.findIndex((v,i)=>v&&!prevB[i]);if(i>=0){gpMap.btn[learn]='b'+i;learn=null;saveGp();renderGp();}prevB=b;return;}
 if(b[1]&&!prevB[1]){halt('Stopped · enable controller to resume');prevB=b;return;}

 if(armed){if(b[4]&&!prevB[4])doAction('vacL');if(b[5]&&!prevB[5])doAction('vacR');for(let i=0;i<b.length;i++)if(i!==1&&i!==2&&i!==4&&i!==5&&b[i]&&!prevB[i])for(const [a,v] of Object.entries(gpMap.btn))if(v==='b'+i)doAction(a);
  if(pendingFrame&&now-pendingAt>550){halt('Controller paused · connection stalled');}else sendFrame(desiredFrame(gp),now);}
 prevB=b;}
addEventListener('gamepadconnected',e=>{if(!armed){GP=e.gamepad.index;renderGp();}});
addEventListener('gamepaddisconnected',e=>{if(e.gamepad.index===GP){GP=null;halt('Controller disconnected');renderGp();}});
addEventListener('blur',()=>{if(armed||arming)halt('Controller paused · page lost focus');});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&(armed||arming))halt('Controller paused · page hidden');});
addEventListener('pagehide',()=>{if(armed||arming){armed=false;fetch('/stopall',{keepalive:true}).catch(()=>{});}});
addEventListener('keydown',e=>{if(e.key==='Escape'){e.preventDefault();halt('Stopped · enable controller to resume');}});
gid('estop').onclick=()=>halt('Stopped · enable controller to resume');
discoverPad();renderGp();load();connectControlSocket();requestAnimationFrame(poll);
</script></body></html>
)rawhtml";

static int axisIndex(const String& n) {
  for (int i = 0; i < NAX; ++i) if (n == axes[i].name) return i;
  return -1;
}

static void handleRoot() { server.send_P(200, "text/html", PAGE); }

static void handleState() {
  String j = "{\"firmware\":\"pnp-toplimit-1\",\"controller\":" + String(controllerArmed ? 1 : 0) + ",\"set\":{";
  j += "\"curMin\":" + String(cfg.curMin) + ",\"curMax\":" + String(cfg.curMax);
  j += ",\"spdMin\":" + String(cfg.spdMin) + ",\"spdMax\":" + String(cfg.spdMax);
  j += ",\"accMin\":" + String(cfg.accMin) + ",\"accMax\":" + String(cfg.accMax);
  j += ",\"turnMin\":" + String(cfg.turnMin) + ",\"turnMax\":" + String(cfg.turnMax);
  j += ",\"spr\":" + String(cfg.stepsPerRev) + "},\"ax\":[";
  for (int i = 0; i < NAX; ++i) {
    Axis& a = axes[i];
    if (i) j += ",";
    j += "{\"n\":\"" + String(a.name) + "\",\"active\":" + (a.active ? 1 : 0) +
         ",\"cur\":" + a.current_mA + ",\"max\":" + a.maxCurrent + ",\"spd\":" + a.speed_hz + ",\"acc\":" + a.accel +
         ",\"mic\":" + a.microsteps + ",\"spread\":" + (a.spread ? 1 : 0) +
         ",\"dir\":" + (a.dir ? 1 : 0) + ",\"en\":" + (a.driverOn ? 1 : 0) +
         ",\"conn\":" + a.conn + ",\"fs\":" + (a.fs ? 1 : 0) +
         ",\"running\":" + (a.fs && a.fs->isRunning() ? 1 : 0) +
         ",\"continuous\":" + (a.fs && a.fs->isRunningContinuously() ? 1 : 0) +
         ",\"target\":" + String((long)(a.fs ? a.fs->targetPos() : 0)) +
         ",\"pos\":" + String((long)(a.fs ? a.fs->getCurrentPosition() : 0));
    if (server.hasArg("drivers")) {
      const uint8_t mres = a.drv->mres();
      j += ",\"hardwareMicro\":" + String(mres <= 8 ? (1U << (8 - mres)) : 0);
    }
    j += "}";
  }
  j += "],\"zlimit\":{\"turns\":" + String(zTravelTurns, 6) + ",\"positiveUp\":" + String(zPositiveUp ? 1 : 0) + ",\"topKnown\":" + String(zTopKnown ? 1 : 0) + "},\"vac\":{\"l\":" + String(vacLeft ? 1 : 0) + ",\"r\":" + String(vacRight ? 1 : 0) +
       "},\"ylink\":" + String(yLinked ? 1 : 0) + "}";
  server.send(200, "application/json", j);
}

static void handleVac() {
  String s = server.arg("side");
  bool on = server.arg("on").toInt() != 0;
  if      (s == "left")  { vacLeft  = on; digitalWrite(PIN_VAC_LEFT,  on ? HIGH : LOW); }
  else if (s == "right") { vacRight = on; digitalWrite(PIN_VAC_RIGHT, on ? HIGH : LOW); }
  else { server.send(400, "text/plain", "bad side"); return; }
  server.send(200, "text/plain", String("Vacuum ") + s + (on ? " ON" : " OFF"));
}

// When Y linking is on, return the sibling index of a Y axis (else -1).
static int ySibling(int i) {
  if (!yLinked) return -1;
  if (String(axes[i].name) == "Y0") return axisIndex("Y1");
  if (String(axes[i].name) == "Y1") return axisIndex("Y0");
  return -1;
}

static bool applyField(Axis& a, const String& f, long v) {
  if      (f == "current") { a.current_mA = constrain(v, 0L, (long)a.maxCurrent); a.drv->rms_current(a.current_mA); }
  else if (f == "micro")   { if (&a == &axes[Z_AXIS] && zLimitsEnabled()) return false; if (!validMicrosteps(v) || (a.fs && a.fs->isRunning())) return false; if (&a == &axes[Z_AXIS] && a.microsteps != v) zTopKnown = false; a.microsteps = v; a.drv->microsteps(a.microsteps == 1 ? 0 : a.microsteps); }
  else if (f == "spread")  { a.spread = v; a.drv->en_spreadCycle(a.spread); }
  else if (f == "accel")   { a.accel = constrain(v, 1L, (long)cfg.accMax); if (a.fs) a.fs->setAcceleration(a.accel); }
  else if (f == "dir")     { a.dir = v; adjustIfSpinning(a); }         // never auto-starts
  else if (f == "speed")   { a.speed_hz = constrain(v, 0L, (long)cfg.spdMax); adjustIfSpinning(a); }
  else if (f == "enabled") { a.driverOn = v; applyDriver(a); if (!a.driverOn) stopAxis(a); }
  else return false;
  return true;
}

static void handleSet() {
  if (controllerArmed) { server.send(409, "text/plain", "Disable controller before manual tuning or moves"); return; }
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  String f = server.arg("field");
  long v = server.arg("value").toInt();
  if (!applyField(axes[i], f, v)) { server.send(400, "text/plain", "bad field"); return; }
  int s = ySibling(i);
  if (s >= 0) applyField(axes[s], f, v);               // mirror to the linked Y sibling
  server.send(200, "text/plain", String(axes[i].name) + (s >= 0 ? "+" + String(axes[s].name) : "") + " " + f + "=" + v);
}

static void handleRun() {
  if (controllerArmed) { server.send(409, "text/plain", "Disable controller before manual tuning or moves"); return; }
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  Axis& a = axes[i];
  if (!a.fs) { server.send(409, "text/plain", String(a.name) + ": no step channel"); return; }
  if (!a.driverOn) { server.send(409, "text/plain", String(a.name) + ": driver OFF"); return; }
  int s = ySibling(i);
  if (s >= 0 && (!axes[s].active || !axes[s].fs || !axes[s].driverOn)) {
    server.send(409, "text/plain", "Both linked Y axes must be active, connected and ON"); return;
  }
  if (i == Z_AXIS && zLimitsEnabled() && !zTopKnown) { server.send(409, "text/plain", "Set Z top zero first"); return; }
  const bool spin = server.arg("mode") == "spin";
  const String value = server.arg("turns");
  char* end = nullptr;
  const float turns = strtof(value.c_str(), &end);
  const double steps = (double)turns * cfg.stepsPerRev * a.microsteps;
  if (!spin && (value.isEmpty() || end == value.c_str() || *end != '\0' ||
                !isfinite(turns) || turns <= 0 || turns > cfg.turnMax ||
                steps < 0.5 || steps > INT32_MAX)) {
    server.send(400, "text/plain", "Turns must be a positive number within the configured limit"); return;
  }
  if (s >= 0) {   // linked: make the sibling match this axis so the pair moves identically
    Axis& b = axes[s];
    b.speed_hz = a.speed_hz; b.accel = a.accel; b.dir = a.dir;
    if (b.microsteps != a.microsteps) { b.microsteps = a.microsteps; b.drv->microsteps(a.microsteps == 1 ? 0 : a.microsteps); }
  }
  String label = String(a.name) + (s >= 0 ? "+" + String(axes[s].name) : "");
  int8_t result = spin ? spinContinuous(a) : runTurns(a, turns);
  if (result == MOVE_OK && s >= 0)
    result = spin ? spinContinuous(axes[s]) : runTurns(axes[s], turns);
  if (result != MOVE_OK) {
    stopAxis(a); if (s >= 0) stopAxis(axes[s]);
    server.send(409, "text/plain", label + " move rejected: " + String(result)); return;
  }
  if (spin) server.send(200, "text/plain", label + " spinning @ " + a.speed_hz + " Hz");
  else server.send(200, "text/plain", label + " running " + String(turns, 2) +
                   " turns (" + String(abs(turnsToSteps(turns, a))) + " pulses @ " +
                   String(turnSpeed(a)) + " Hz)");
}

static void handleStop() {
  if (controllerArmed) { releaseController(); server.send(200, "text/plain", "Controller stopped"); return; }
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  stopAxis(axes[i]);
  int s = ySibling(i);
  if (s >= 0) stopAxis(axes[s]);
  server.send(200, "text/plain", String(axes[i].name) + " stop");
}

static void handlePair() {
  if (controllerArmed) { server.send(409, "text/plain", "Disable controller before manual tuning or moves"); return; }
  stopAxis(axes[axisIndex("Y0")]); stopAxis(axes[axisIndex("Y1")]);
  yLinked = server.arg("on").toInt() != 0;
  saveSettings();
  server.send(200, "text/plain", yLinked ? "Y0 + Y1 linked" : "Y0 / Y1 independent");
}

// Continuous jog while a control is held (dir 1/-1 run, 0 stop). Uses its own
// speed (hz) so it does NOT clobber the slider's speed_hz.
static void handleJog() {
  if (controllerArmed) { server.send(409, "text/plain", "Disable controller before manual tuning or moves"); return; }
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  int dir = server.arg("dir").toInt();
  long hz = server.arg("hz").toInt();
  int s = ySibling(i); int idxs[2] = {i, s};
  for (int n = 0; n < 2; n++) {
    int k = idxs[n]; if (k < 0) continue;
    Axis& a = axes[k];
    if (!a.fs || !a.driverOn) continue;
    if (dir == 0) { a.fs->stopMove(); a.spinning = false; }
    else { a.fs->setAcceleration(a.accel); a.fs->setSpeedInHz(max<long>(hz, 1L));
           if (k == Z_AXIS) { if (startZRate(dir > 0 ? max<long>(hz, 1L) : -max<long>(hz, 1L)) != MOVE_OK) { server.send(409, "text/plain", "Set Z top zero first"); return; } }
           else if (dir > 0) a.fs->runForward(); else a.fs->runBackward(); }
  }
  server.send(200, "text/plain", "jog");
}

// Absolute move (for macro recall).
static void handleGoto() {
  if (controllerArmed) { server.send(409, "text/plain", "Disable controller before manual tuning or moves"); return; }
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  long pos;
  if (!integerArg("pos", INT32_MIN, INT32_MAX, pos)) { server.send(400, "text/plain", "Invalid position"); return; }
  if (i == Z_AXIS && !zTargetAllowed(pos)) { server.send(409, "text/plain", "Z target outside limits or top zero missing"); return; }
  int s = ySibling(i); int idxs[2] = {i, s};
  for (int n = 0; n < 2; n++) {
    int k = idxs[n]; if (k < 0) continue;
    Axis& a = axes[k];
    if (!a.fs || !a.driverOn) continue;
    a.spinning = false;
    a.fs->setAcceleration(a.accel);
    a.fs->setSpeedInHz(max<uint32_t>(a.speed_hz, 1));
    a.fs->moveTo(pos);
  }
  server.send(200, "text/plain", "goto " + String(pos));
}

static void handleZero() {
  if (controllerArmed) { server.send(409, "text/plain", "Disable controller before manual tuning or moves"); return; }
  String ax = server.arg("axis");
  if (zLimitsEnabled() && (ax == "all" || ax == "Z")) { server.send(409, "text/plain", "Use Set current Z as top instead of zeroing limited Z"); return; }
  for (const auto& a : axes) if ((ax == "all" || ax == a.name) && a.fs && a.fs->isRunning()) { server.send(409, "text/plain", "Stop motors before zeroing"); return; }
  for (int i = 0; i < NAX; i++)
    if ((ax == "all" || ax == axes[i].name) && axes[i].fs) axes[i].fs->setCurrentPosition(0);
  server.send(200, "text/plain", "zeroed " + ax);
}

static void handleStopAll() {
  releaseController();
  server.send(200, "text/plain", "ALL STOP");
}

static void handleSettings() {
  if (controllerArmed) { server.send(409, "text/plain", "Disable controller before manual tuning or moves"); return; }
  auto getU = [](const char* k, uint32_t def) {
    return server.hasArg(k) ? (uint32_t)server.arg(k).toInt() : def;
  };
  cfg.curMin = getU("curMin", cfg.curMin);   cfg.curMax = getU("curMax", cfg.curMax);
  cfg.spdMin = getU("spdMin", cfg.spdMin);   cfg.spdMax = getU("spdMax", cfg.spdMax);
  cfg.accMin = getU("accMin", cfg.accMin);   cfg.accMax = getU("accMax", cfg.accMax);
  cfg.turnMin = getU("turnMin", cfg.turnMin); cfg.turnMax = getU("turnMax", cfg.turnMax);
  const uint32_t spr = getU("spr", cfg.stepsPerRev);
  if (!spr || spr > 65535 || (double)zTravelTurns * spr > INT32_MAX / 256) { server.send(400, "text/plain", "Invalid steps per revolution for Z limits"); return; }
  if (cfg.stepsPerRev != spr) zTopKnown = false;
  cfg.stepsPerRev = spr;
  for (int i = 0; i < NAX; ++i) {
    bool was = axes[i].active;
    axes[i].active = getU((String("ax") + axes[i].name).c_str(), was) != 0;
    if (axes[i].active && !was) {        // newly activated: connect + configure
      connectStepper(axes[i]);
      initDriver(axes[i].drv);
      axes[i].conn = axes[i].drv->test_connection();
      applyDriver(axes[i]);
    }
    if (!axes[i].active && was) { axes[i].driverOn = false; axes[i].drv->toff(0); stopAxis(axes[i]); }
  }
  saveSettings();
  server.send(200, "text/plain", "settings saved");
}

// Generate small test moves with all motor outputs physically disabled by EN.
// Leave every software driver OFF afterwards; this is an explicit diagnostic API.
static volatile uint32_t testPulses[NAX] = {};
static void IRAM_ATTR countTestPulse(void* arg) { ++*static_cast<volatile uint32_t*>(arg); }
static void handleMotionTest() {
  releaseController();
  const bool fastTest = server.arg("fast") == "1";
  const int32_t testSteps = fastTest ? 3200 : 320;
  digitalWrite(PIN_EN, HIGH);
  int32_t originalPositions[NAX] = {};
  for (int i = 0; i < NAX; ++i)
    if (axes[i].fs) originalPositions[i] = axes[i].fs->getCurrentPosition();
  for (auto& a : axes) {
    if (a.fs) a.fs->forceStopAndNewPosition(a.fs->getCurrentPosition());
    a.driverOn = false; a.spinning = false; a.drv->toff(0);
  }
  for (int i = 0; i < NAX; ++i) {
    testPulses[i] = 0;
    attachInterruptArg(axes[i].stepPin, countTestPulse, (void*)&testPulses[i], RISING);
  }
  String j = "[";
  for (int test = 0; test < NAX + 1; ++test) {
    int first = test < NAX ? test : axisIndex("Y0");
    int second = test < NAX ? -1 : axisIndex("Y1");
    int ids[2] = {first, second};
    int32_t start[2] = {0, 0};
    for (int k = 0; k < 2; ++k) {
      if (ids[k] < 0 || !axes[ids[k]].fs) continue;
      auto* fs = axes[ids[k]].fs;
      start[k] = fs->getCurrentPosition();
      testPulses[ids[k]] = 0;
      fs->setAcceleration(fastTest ? 30000 : 2000); fs->setSpeedInHz(fastTest ? 4000 : 400); fs->move(testSteps);
    }
    uint32_t began = millis();
    delay(20);
    while (millis() - began < 2000) {
      bool running = false;
      for (int k = 0; k < 2; ++k)
        if (ids[k] >= 0 && axes[ids[k]].fs) running |= axes[ids[k]].fs->isRunning();
      if (!running) break;
      delay(1);
    }
    for (int k = 0; k < 2; ++k) {
      if (ids[k] < 0) continue;
      auto& a = axes[ids[k]];
      if (j.length() > 1) j += ",";
      j += "{\"axis\":\"" + String(a.name) + "\",\"paired\":" + String(test == NAX ? 1 : 0);
      j += ",\"running\":" + String(a.fs && a.fs->isRunning() ? 1 : 0);
      j += ",\"pulses\":" + String(testPulses[ids[k]]);
      j += ",\"delta\":" + String((long)(a.fs ? a.fs->getCurrentPosition() - start[k] : 0)) + "}";
      if (a.fs) a.fs->forceStopAndNewPosition(a.fs->getCurrentPosition());
    }
  }
  for (int i = 0; i < NAX; ++i) {
    detachInterrupt(axes[i].stepPin);
    if (axes[i].fs) axes[i].fs->setCurrentPosition(originalPositions[i]);
  }
  // Safe to release EN only once every pulse queue has been cleared and toff=0.
  digitalWrite(PIN_EN, LOW);
  server.send(200, "application/json", j + "]");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  loadSettings();

  TMCSerialA.begin(UART_BAUD, SERIAL_8N1, PIN_UART_A, PIN_UART_TX_A);
  TMCSerialB.begin(UART_BAUD, SERIAL_8N1, PIN_UART_B, PIN_UART_TX_B);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);                 // keep all drivers disabled during setup

  pinMode(PIN_VAC_LEFT, OUTPUT);  digitalWrite(PIN_VAC_LEFT, LOW);   // vacuums OFF at boot
  pinMode(PIN_VAC_RIGHT, OUTPUT); digitalWrite(PIN_VAC_RIGHT, LOW);

  engine.init();
  engine.task_rate(2);

  for (int i = 0; i < NAX; ++i) {
    Axis& a = axes[i];
    if (!a.active) continue;                  // inactive axes stay disconnected
    connectStepper(a);
    initDriver(a.drv);
    a.conn = a.drv->test_connection();
    applyDriver(a);                           // boots with driverOn default + speed 0 => no motion
  }

  digitalWrite(PIN_EN, LOW);                  // all configured drivers are OFF

  // Try to join your network; fall back to our own AP if it doesn't connect.
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Joining WiFi '%s' ...\n", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_JOIN_TIMEOUT_MS) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected -> http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("Join failed; AP '%s' -> http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }

  // Over-the-air firmware updates (password-protected).
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() {                 // stop all motion before reflashing
    for (auto& a : axes) stopAxis(a);
    digitalWrite(PIN_EN, HIGH);
    digitalWrite(PIN_VAC_LEFT, LOW); digitalWrite(PIN_VAC_RIGHT, LOW);
  });
  ArduinoOTA.begin();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/state", HTTP_GET, handleState);
  server.on("/zlimits", HTTP_POST, handleZLimits);
  server.on("/controller", HTTP_POST, handleController);
  server.on("/control", HTTP_POST, handleControlFrame);
  server.on("/motiontest", HTTP_POST, handleMotionTest);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/run", HTTP_GET, handleRun);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/vac", HTTP_GET, handleVac);
  server.on("/pair", HTTP_GET, handlePair);
  server.on("/jog", HTTP_GET, handleJog);
  server.on("/goto", HTTP_GET, handleGoto);
  server.on("/zero", HTTP_GET, handleZero);
  server.on("/stopall", HTTP_GET, handleStopAll);
  server.begin();
  controlSocket.begin();
  controlSocket.onEvent(controlSocketEvent);
}

void loop() {
  if (controllerArmed && millis() - controllerLastFrame > (controllerPrimed ? CONTROLLER_TIMEOUT_MS : 2000)) releaseController();
  controlSocket.loop();
  serviceControllerResolution();
  server.handleClient();
  ArduinoOTA.handle();
}
