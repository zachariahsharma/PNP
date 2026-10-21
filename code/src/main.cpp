/*
 * PNP board — 5-axis TMC2209 tuning + turn-test panel over WiFi.
 *
 * Target: ESP32-S3-DevKitC (on the PNP PCB)
 *
 * Power from the 24 V source, join WiFi AP "PNP-Control", open http://192.168.4.1.
 * No USB needed while running (avoids the 24V-vs-USB 3.3V contention). Flash with
 * USB only while the 24 V is OFF.
 *
 * Axes (traced from PNP.kicad_pcb). EN = GPIO48 (global, held low; per-axis on/off
 * via TMC `toff`). Two UART buses; address set by each socket's MS1/MS2 straps:
 *   Bus A = GPIO8  (1k series R):  C1=addr1(step6,dir5)  Z=addr0(step10,dir9)  X=addr2(step13,dir14)
 *   Bus B = GPIO39 (1k series R):  Y0=addr2(step2,dir1)  Y1=addr1(step47,dir21)
 *
 * Step/dir pulses: FastAccelStepper (hardware-timed, with acceleration + bounded
 * "move N turns"). Current/microstep/chopper: TMC2209 over UART.
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ---- WiFi AP ----
static constexpr char AP_SSID[] = "PNP-Control";
static constexpr char AP_PASS[] = "pnp12345";   // >= 8 chars

// ---- UART buses ----
static constexpr int      PIN_UART_A = 8;        // C1, Z, X
static constexpr int      PIN_UART_B = 39;       // Y0, Y1
static constexpr uint32_t UART_BAUD  = 115200;
HardwareSerial TMCSerialA(1);
HardwareSerial TMCSerialB(2);

static constexpr float R_SENSE = 0.11f;          // **match your driver module**
static constexpr int   PIN_EN  = 48;             // global enable, active LOW

// ---- Vacuum low-side switches (active HIGH; 10k gate pulldowns keep them OFF) ----
static constexpr int PIN_VAC_LEFT  = 7;          // Q1 gate via R5  (Vacuum Left)
static constexpr int PIN_VAC_RIGHT = 4;          // Q3 gate via R12 (Vacuum Right)
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
  {"C1", &drvC1,  6,   5,   nullptr, true,  340,  424, 0, 2000, 16, true, true, false, 0}, // Nema8 hollow 0.6A
  {"Z",  &drvZ,  10,   9,   nullptr, true,  1200, 1485, 0, 2000, 16, true, true, false, 0}, // Nema17 2.1A
  {"X",  &drvX,  13,  14,   nullptr, false, 1200, 1485, 0, 2000, 16, true, true, false, 0}, // Nema17 2.1A
  {"Y0", &drvY0,  2,   1,   nullptr, false, 1200, 1485, 0, 2000, 16, true, true, false, 0}, // Nema17 2.1A
  {"Y1", &drvY1, 47,  21,   nullptr, false, 1200, 1485, 0, 2000, 16, true, true, false, 0}, // Nema17 2.1A
  {"C0", &drvC0, 41,  42,   nullptr, false, 340,  424, 0, 2000, 16, true, true, false, 0}, // Nema8 hollow 0.6A
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

Preferences prefs;
WebServer server(80);

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
  prefs.end();
}

// ---- Driver / motion control ----
static void connectStepper(Axis& a) {
  if (a.fs) return;
  a.fs = engine.stepperConnectToPin(a.stepPin);
  if (a.fs) {
    a.fs->setDirectionPin(a.dirPin);
    a.fs->setAutoEnable(false);          // EN is shared, managed manually
    a.fs->setAcceleration(a.accel);
    a.fs->setSpeedInHz(max<uint32_t>(a.speed_hz, 1));
  }
}

static void initDriver(TMC2209Stepper* d) {
  d->begin();
  d->pdn_disable(true);
  d->I_scale_analog(false);   // ignore VREF pot; current is software-defined
  d->mstep_reg_select(true);
  d->pwm_autoscale(true);
}

// Push current/microstep/chopper/enable to the TMC.
static void applyDriver(Axis& a) {
  a.drv->rms_current(a.current_mA);
  a.drv->microsteps(a.microsteps);
  a.drv->en_spreadCycle(a.spread);
  a.drv->toff(a.driverOn ? 4 : 0);
}

static int32_t turnsToSteps(float turns, const Axis& a) {
  return (int32_t)lround(turns * (float)cfg.stepsPerRev * (float)a.microsteps);
}

static void spinContinuous(Axis& a) {
  if (!a.fs || !a.driverOn || a.speed_hz < 1) { if (a.fs) a.fs->stopMove(); a.spinning = false; return; }
  a.fs->setAcceleration(a.accel);
  a.fs->setSpeedInHz(a.speed_hz);
  if (a.dir) a.fs->runForward(); else a.fs->runBackward();
  a.spinning = true;
}

// Re-apply speed/direction to a motor that is ALREADY spinning — never starts it.
static void adjustIfSpinning(Axis& a) {
  if (!a.fs || !a.spinning) return;
  a.fs->setSpeedInHz(max<uint32_t>(a.speed_hz, 1));
  if (a.dir) a.fs->runForward(); else a.fs->runBackward();
}

static void runTurns(Axis& a, float turns) {
  if (!a.fs || !a.driverOn) return;
  a.spinning = false;
  a.fs->setAcceleration(a.accel);
  // If no usable speed is set, run the move at a sane default instead of
  // crawling at 1 Hz (which looks like it never finishes).
  a.fs->setSpeedInHz(a.speed_hz < 50 ? 400 : a.speed_hz);
  int32_t steps = turnsToSteps(turns, a);
  if (!a.dir) steps = -steps;
  a.fs->move(steps);
}

// ---- Web UI ----
static const char PAGE[] PROGMEM = R"rawhtml(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PNP Driver Tuner</title>
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
</style></head><body>
<header><b>PNP Driver Tuner</b><div><button onclick="toggleGp()">🎮 Gamepad</button> <button onclick="toggleSettings()">⚙ Settings</button></div></header>
<div id="settings"></div>
<div id="gppanel" style="display:none;padding:14px"></div>
<div style="padding:8px 18px;color:#9aa0aa;font-size:13px;line-height:1.5">
 Set values, switch a <b>Driver ON</b>, then <b>Spin ∞</b> (continuous) or <b>Run turns</b> (move a set number of turns). <b>Stop</b> halts instantly.<br>
 <b>⚙ Settings</b>: activate axes & set slider limits · <b>🎮 Gamepad</b>: drive it with an Xbox controller.
</div>
<div style="padding:4px 18px 10px"><button id="estop" class="estop">■ STOP ALL</button></div>
<div class="wrap" id="vacwrap"></div>
<div class="wrap" id="wrap"></div>
<div class="status" id="status">loading…</div>
<script>
const MS=[1,2,4,8,16,32,64,128,256];
const MOTOR={C0:'Nema 8 · 0.6A',C1:'Nema 8 · 0.6A',X:'Nema 17 · 2.1A',Y0:'Nema 17 · 2.1A',Y1:'Nema 17 · 2.1A',Z:'Nema 17 · 2.1A'};
const status=document.getElementById('status');
let S={}, AX=[], VAC={l:0,r:0}, YLINK=0;
function api(u){return fetch(u).then(r=>r.text()).then(t=>{status.textContent=t;});}
function act(on,off){on.classList.add('on');off.classList.remove('on');}

function renderVac(){
 document.getElementById('vacwrap').innerHTML=`
  <div class="card"><h2>Vacuum</h2>
   <div class="row"><label>Vacuum Left (GPIO7)</label>
    <div class="toggle"><button id="vl0">OFF</button><button id="vl1">ON</button></div></div>
   <div class="row"><label>Vacuum Right (GPIO4)</label>
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
function toggleSettings(){const e=gid('settings');e.style.display=e.style.display=='none'||!e.style.display?'block':'none';}
function saveSettings(){
 const q=new URLSearchParams();
 ['cur','spd','acc','turn'].forEach(k=>{q.set(k+'Min',gid(k+'Min').value);q.set(k+'Max',gid(k+'Max').value);});
 q.set('spr',gid('spr').value);
 AX.forEach(a=>q.set('ax'+a.n,gid('ax'+a.n).checked?1:0));
 api('/settings?'+q.toString()).then(()=>load());
}
function load(){fetch('/state').then(r=>r.json()).then(d=>{S=d.set;AX=d.ax;VAC=d.vac;YLINK=d.ylink;render();});}
load();

// ---------- Xbox / Gamepad control ----------
let GP=null, learn=null;
let gpMap=JSON.parse(localStorage.getItem('pnpGpMap')||'{}'); gpMap.ax=gpMap.ax||{}; gpMap.btn=gpMap.btn||{};
let macros=JSON.parse(localStorage.getItem('pnpMacros')||'{}');
const ACTIONS=[['stopAll','STOP ALL'],['vacL','Vacuum L toggle'],['vacR','Vacuum R toggle'],['m1go','Go Macro 1'],['m2go','Go Macro 2'],['m3go','Go Macro 3'],['zero','Zero all']];
const SCH_DEF={xy:{fast:3000,slow:200,dz:0.18,micro:16,accel:20000,invX:false,invY:true},z:{fast:1500,slow:150,dz:0.12,micro:16,accel:15000,invert:false},c:{fast:2000,slow:150,dz:0.18,micro:16,accel:15000,invert:false}};
let scheme=JSON.parse(localStorage.getItem('pnpScheme')||'{}');
scheme.xy=Object.assign({},SCH_DEF.xy,scheme.xy);scheme.z=Object.assign({},SCH_DEF.z,scheme.z);scheme.c=Object.assign({},SCH_DEF.c,scheme.c);
let flight=localStorage.getItem('pnpFlight')==='1', cActive='C1', prevA=false, prevLB=false, prevRB=false, paused=false;
function saveSch(){localStorage.setItem('pnpScheme',JSON.stringify(scheme));}
function grp(g,label,extra){const o=scheme[g];return `<h3>${label}</h3><div class="row">fast <input type="number" class="sc" data-g="${g}" data-k="fast" value="${o.fast}" style="width:80px"> Hz &nbsp; slow <input type="number" class="sc" data-g="${g}" data-k="slow" value="${o.slow}" style="width:70px"> Hz &nbsp; deadzone <input type="number" step="0.05" class="sc" data-g="${g}" data-k="dz" value="${o.dz}" style="width:60px"> &nbsp; µsteps <select class="sc" data-g="${g}" data-k="micro">${MS.map(x=>`<option ${o.micro==x?'selected':''}>${x}</option>`).join('')}</select> &nbsp; accel <input type="number" class="sc" data-g="${g}" data-k="accel" value="${o.accel}" style="width:90px"> st/s² &nbsp; ${extra}</div>`;}
function schemeHtml(){return grp('xy','Left stick → X (horizontal) / Y0+Y1 (vertical)',`<label><input type="checkbox" class="sc" data-g="xy" data-k="invX" ${scheme.xy.invX?'checked':''}>invX</label> <label><input type="checkbox" class="sc" data-g="xy" data-k="invY" ${scheme.xy.invY?'checked':''}>invY</label>`)+grp('z','Triggers → Z  (RT = up, LT = down)',`<label><input type="checkbox" class="sc" data-g="z" data-k="invert" ${scheme.z.invert?'checked':''}>invert</label>`)+grp('c','Right stick → C0 / C1  (A switches)',`<label><input type="checkbox" class="sc" data-g="c" data-k="invert" ${scheme.c.invert?'checked':''}>invert</label>`);}
function applyScheme(){const push=(ns,g)=>ns.forEach(n=>{api(`/set?axis=${n}&field=micro&value=${scheme[g].micro}`);api(`/set?axis=${n}&field=accel&value=${scheme[g].accel}`);});push(['X','Y0','Y1'],'xy');push(['Z'],'z');push(['C0','C1'],'c');}
function qhz(v,cfg){const a=Math.abs(v);if(a<cfg.dz)return 0;const t=(a-cfg.dz)/(1-cfg.dz);return Math.max(cfg.slow,Math.round((cfg.slow+t*(cfg.fast-cfg.slow))/50)*50);}
function fjog(name,v,cfg,inv){let vv=inv?-v:v;const hz=qhz(vv,cfg);const dir=hz===0?0:(vv>0?1:-1);const key=dir+':'+hz;if(lastJog[name]!==key){jog(name,dir,hz);lastJog[name]=key;}}
function fjogPair(ns,v,cfg,inv){let vv=inv?-v:v;const hz=qhz(vv,cfg);const dir=hz===0?0:(vv>0?1:-1);const key=dir+':'+hz;if(lastJog['Yp']!==key){ns.forEach(n=>jog(n,dir,hz));lastJog['Yp']=key;}}
function flightPoll(gp){const c0=Math.abs(gp.axes[0]||0),c1=Math.abs(gp.axes[1]||0),c2=Math.abs(gp.axes[2]||0),trg=(gp.buttons[6]?gp.buttons[6].value:0)+(gp.buttons[7]?gp.buttons[7].value:0);if(paused){if(c0<0.25&&c1<0.25&&c2<0.25&&trg<0.15)paused=false;else return;}fjog('X',gp.axes[0]||0,scheme.xy,scheme.xy.invX);fjogPair(['Y0','Y1'],gp.axes[1]||0,scheme.xy,scheme.xy.invY);const rt=gp.buttons[7]?gp.buttons[7].value:0,lt=gp.buttons[6]?gp.buttons[6].value:0;fjog('Z',rt-lt,scheme.z,scheme.z.invert);fjog(cActive,gp.axes[2]||0,scheme.c,scheme.c.invert);const aNow=gp.buttons[0]&&(gp.buttons[0].pressed||gp.buttons[0].value>0.5);if(aNow&&!prevA){const old=cActive;cActive=(cActive==='C1')?'C0':'C1';jog(old,0,0);lastJog[old]='x';const ce=gid('cact');if(ce)ce.textContent=cActive;status.textContent='C stick → '+cActive;}prevA=aNow;const lb=gp.buttons[4]&&gp.buttons[4].pressed;if(lb&&!prevLB)doAction('vacL');prevLB=lb;const rb=gp.buttons[5]&&gp.buttons[5].pressed;if(rb&&!prevRB)doAction('vacR');prevRB=rb;}
function saveGp(){localStorage.setItem('pnpGpMap',JSON.stringify(gpMap));}
function saveMac(){localStorage.setItem('pnpMacros',JSON.stringify(macros));}
function toggleGp(){const e=gid('gppanel');e.style.display=(e.style.display=='none'||!e.style.display)?'block':'none';if(e.style.display=='block')renderGp();}
addEventListener('gamepadconnected',e=>{GP=e.gamepad.index;renderGp();});
addEventListener('gamepaddisconnected',()=>{GP=null;renderGp();});

function renderGp(){
 const el=gid('gppanel'); if(!el) return;
 const pads=navigator.getGamepads?navigator.getGamepads():[];
 const gp=GP!=null?pads[GP]:null;
 const names=AX.filter(a=>a.active).map(a=>a.n);
 let h=`<h2 style="margin:4px 0">Xbox controller</h2>
  <div style="color:#9aa0aa;margin-bottom:10px">${gp?('✓ '+gp.id):'Not connected — plug in / pair your controller and press any button.'}</div>`;
 if(gp){
  h+=`<div class="row"><label style="display:flex;gap:6px;align-items:center"><input type="checkbox" id="flight" ${flight?'checked':''}> <b>PNP flight mode</b> — L-stick = X/Y gantry · triggers = Z · R-stick = C0/C1 (A switches) · LB/RB = vacuums</label></div>`;
  if(flight){
   h+=schemeHtml();
   h+=`<div class="row">Right stick driving: <b id="cact">${cActive}</b>  (press <b>A</b> to switch)</div>`;
   h+=`<div style="color:#9aa0aa;font-size:12px;margin:6px 0">Activate X, Y0, Y1, Z, C0, C1 in ⚙ Settings. Turning flight ON enables their drivers.</div>`;
  } else {
   h+=`<h3>Analog jog — sticks & triggers</h3>`;
   for(let i=0;i<gp.axes.length;i++){const m=gpMap.ax[i]||{};
    h+=`<div class="row">Axis ${i}
     <select class="gpd" data-i="${i}"><option value="">—none—</option>${names.map(n=>`<option ${m.dest===n?'selected':''}>${n}</option>`).join('')}</select>
     max <input type="number" class="gps" data-i="${i}" value="${m.scale||2000}" style="width:76px"> Hz
     dz <input type="number" step="0.05" class="gpz" data-i="${i}" value="${m.dz||0.15}" style="width:58px">
     <label><input type="checkbox" class="gpi" data-i="${i}" ${m.inv?'checked':''}>inv</label></div>`;}
  }
  h+=`<h3>Buttons & triggers → actions</h3>`;
  h+=ACTIONS.map(a=>`<div class="row">${a[1]}: <b>${gpMap.btn[a[0]]||'(unbound)'}</b>
    <button class="gpl" data-a="${a[0]}">${learn===a[0]?'press control…':'Learn'}</button></div>`).join('');
  h+=`<h3>Macros — saved positions</h3>`;
  h+=[1,2,3].map(s=>`<div class="row">Macro ${s}: <span style="color:#9aa0aa">${macros[s]?Object.entries(macros[s]).map(([k,v])=>k+'='+v).join(', '):'empty'}</span>
    <button class="gpm" data-s="${s}">Save current</button></div>`).join('');
 }
 el.innerHTML=h;
 el.querySelectorAll('.gpd').forEach(x=>x.onchange=e=>{let i=e.target.dataset.i;(gpMap.ax[i]=gpMap.ax[i]||{}).dest=e.target.value;saveGp();});
 el.querySelectorAll('.gps').forEach(x=>x.onchange=e=>{let i=e.target.dataset.i;(gpMap.ax[i]=gpMap.ax[i]||{}).scale=+e.target.value;saveGp();});
 el.querySelectorAll('.gpz').forEach(x=>x.onchange=e=>{let i=e.target.dataset.i;(gpMap.ax[i]=gpMap.ax[i]||{}).dz=+e.target.value;saveGp();});
 el.querySelectorAll('.gpi').forEach(x=>x.onchange=e=>{let i=e.target.dataset.i;(gpMap.ax[i]=gpMap.ax[i]||{}).inv=e.target.checked;saveGp();});
 el.querySelectorAll('.gpl').forEach(x=>x.onclick=e=>{learn=e.target.dataset.a;renderGp();});
 el.querySelectorAll('.gpm').forEach(x=>x.onclick=e=>saveMacro(e.target.dataset.s));
 el.querySelectorAll('.sc').forEach(x=>x.onchange=e=>{const g=e.target.dataset.g,k=e.target.dataset.k;scheme[g][k]=e.target.type==='checkbox'?e.target.checked:+e.target.value;saveSch();if(k==='micro'||k==='accel')applyScheme();});
 const fl=gid('flight'); if(fl) fl.onchange=e=>{flight=e.target.checked;localStorage.setItem('pnpFlight',flight?'1':'0');lastJog={};['X','Y0','Y1','Z','C0','C1'].forEach(n=>jog(n,0,0));if(flight){applyScheme();['X','Y0','Y1','Z','C0','C1'].forEach(n=>api(`/set?axis=${n}&field=enabled&value=1`));}renderGp();};
}
function jog(n,dir,hz){api(`/jog?axis=${n}&dir=${dir}&hz=${hz}`);}
function saveMacro(s){fetch('/state').then(r=>r.json()).then(d=>{const m={};d.ax.forEach(a=>{if(a.active)m[a.n]=a.pos;});macros[s]=m;saveMac();renderGp();});}
function gotoMacro(s){const m=macros[s];if(!m)return;Object.keys(m).forEach(n=>api(`/goto?axis=${n}&pos=${m[n]}`));}
function doAction(a){
 if(a==='stopAll')api('/stopall');
 else if(a==='vacL'){VAC.l=VAC.l?0:1;api('/vac?side=left&on='+VAC.l);}
 else if(a==='vacR'){VAC.r=VAC.r?0:1;api('/vac?side=right&on='+VAC.r);}
 else if(a==='zero')api('/zero?axis=all');
 else if(a[0]==='m')gotoMacro(a[1]);
}
let prevB=[], lastJog={};
function poll(){requestAnimationFrame(poll);
 const pads=navigator.getGamepads?navigator.getGamepads():[]; const gp=GP!=null?pads[GP]:null; if(!gp)return;
 if(learn){for(let i=0;i<gp.buttons.length;i++){if(gp.buttons[i].pressed||gp.buttons[i].value>0.5){gpMap.btn[learn]='b'+i;learn=null;saveGp();renderGp();return;}}}
 if(flight){flightPoll(gp);}
 else{Object.keys(gpMap.ax).forEach(i=>{const m=gpMap.ax[i];if(!m||!m.dest)return;
  let v=gp.axes[i]||0; if(m.inv)v=-v; const dz=m.dz||0.15; const n=m.dest;
  if(Math.abs(v)<dz){if(lastJog[n]!==0){jog(n,0,0);lastJog[n]=0;}return;}
  const hz=Math.max(200,Math.round(Math.abs(v)*(m.scale||2000)/200)*200); const dir=v>0?1:-1; const key=dir+':'+hz;
  if(lastJog[n]!==key){jog(n,dir,hz);lastJog[n]=key;}});}
 for(let i=0;i<gp.buttons.length;i++){const p=gp.buttons[i].pressed||gp.buttons[i].value>0.5;
  if(p&&!prevB[i]){Object.keys(gpMap.btn).forEach(act=>{if(gpMap.btn[act]==='b'+i)doAction(act);});}
  prevB[i]=p;}
}
{const es=gid('estop'); if(es) es.onclick=()=>{api('/stopall');paused=true;lastJog={};status.textContent='STOP ALL — flight paused until sticks re-center';};}
poll();
</script></body></html>
)rawhtml";

static int axisIndex(const String& n) {
  for (int i = 0; i < NAX; ++i) if (n == axes[i].name) return i;
  return -1;
}

static void handleRoot() { server.send_P(200, "text/html", PAGE); }

static void handleState() {
  String j = "{\"set\":{";
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
         ",\"pos\":" + String((long)(a.fs ? a.fs->getCurrentPosition() : 0)) + "}";
  }
  j += "],\"vac\":{\"l\":" + String(vacLeft ? 1 : 0) + ",\"r\":" + String(vacRight ? 1 : 0) +
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
  else if (f == "micro")   { a.microsteps = v; a.drv->microsteps(a.microsteps); }
  else if (f == "spread")  { a.spread = v; a.drv->en_spreadCycle(a.spread); }
  else if (f == "accel")   { a.accel = constrain(v, 1L, (long)cfg.accMax); if (a.fs) a.fs->setAcceleration(a.accel); }
  else if (f == "dir")     { a.dir = v; adjustIfSpinning(a); }         // never auto-starts
  else if (f == "speed")   { a.speed_hz = constrain(v, 0L, (long)cfg.spdMax); adjustIfSpinning(a); }
  else if (f == "enabled") { a.driverOn = v; applyDriver(a); if (!a.driverOn && a.fs) { a.fs->forceStop(); a.spinning = false; } }
  else return false;
  return true;
}

static void handleSet() {
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
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  Axis& a = axes[i];
  if (!a.fs) { server.send(409, "text/plain", String(a.name) + ": no step channel"); return; }
  if (!a.driverOn) { server.send(409, "text/plain", String(a.name) + ": driver OFF"); return; }
  int s = ySibling(i);                                 // sibling self-guards (fs/driverOn)
  if (s >= 0) {   // linked: make the sibling match this axis so the pair moves identically
    Axis& b = axes[s];
    b.speed_hz = a.speed_hz; b.accel = a.accel; b.dir = a.dir;
    if (b.microsteps != a.microsteps) { b.microsteps = a.microsteps; b.drv->microsteps(a.microsteps); }
  }
  String label = String(a.name) + (s >= 0 ? "+" + String(axes[s].name) : "");
  if (server.arg("mode") == "spin") {
    spinContinuous(a);
    if (s >= 0) spinContinuous(axes[s]);
    server.send(200, "text/plain", label + " spinning @ " + a.speed_hz + " Hz");
  } else {
    float turns = server.arg("turns").toFloat();
    runTurns(a, turns);
    if (s >= 0) runTurns(axes[s], turns);
    server.send(200, "text/plain", label + " running " + String(turns, 2) + " turns");
  }
}

static void handleStop() {
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  if (axes[i].fs) axes[i].fs->forceStop();   // immediate, no deceleration
  axes[i].spinning = false;
  int s = ySibling(i);
  if (s >= 0) { if (axes[s].fs) axes[s].fs->forceStop(); axes[s].spinning = false; }
  server.send(200, "text/plain", String(axes[i].name) + " stop");
}

static void handlePair() {
  yLinked = server.arg("on").toInt() != 0;
  saveSettings();
  server.send(200, "text/plain", yLinked ? "Y0 + Y1 linked" : "Y0 / Y1 independent");
}

// Continuous jog while a control is held (dir 1/-1 run, 0 stop). Uses its own
// speed (hz) so it does NOT clobber the slider's speed_hz.
static void handleJog() {
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
           if (dir > 0) a.fs->runForward(); else a.fs->runBackward(); }
  }
  server.send(200, "text/plain", "jog");
}

// Absolute move (for macro recall).
static void handleGoto() {
  int i = axisIndex(server.arg("axis"));
  if (i < 0) { server.send(400, "text/plain", "bad axis"); return; }
  long pos = server.arg("pos").toInt();
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
  String ax = server.arg("axis");
  for (int i = 0; i < NAX; i++)
    if ((ax == "all" || ax == axes[i].name) && axes[i].fs) axes[i].fs->setCurrentPosition(0);
  server.send(200, "text/plain", "zeroed " + ax);
}

static void handleStopAll() {
  for (int i = 0; i < NAX; i++) { if (axes[i].fs) axes[i].fs->forceStop(); axes[i].spinning = false; }
  server.send(200, "text/plain", "ALL STOP");
}

static void handleSettings() {
  auto getU = [](const char* k, uint32_t def) {
    return server.hasArg(k) ? (uint32_t)server.arg(k).toInt() : def;
  };
  cfg.curMin = getU("curMin", cfg.curMin);   cfg.curMax = getU("curMax", cfg.curMax);
  cfg.spdMin = getU("spdMin", cfg.spdMin);   cfg.spdMax = getU("spdMax", cfg.spdMax);
  cfg.accMin = getU("accMin", cfg.accMin);   cfg.accMax = getU("accMax", cfg.accMax);
  cfg.turnMin = getU("turnMin", cfg.turnMin); cfg.turnMax = getU("turnMax", cfg.turnMax);
  cfg.stepsPerRev = getU("spr", cfg.stepsPerRev);
  for (int i = 0; i < NAX; ++i) {
    bool was = axes[i].active;
    axes[i].active = getU((String("ax") + axes[i].name).c_str(), was) != 0;
    if (axes[i].active && !was) {        // newly activated: connect + configure
      connectStepper(axes[i]);
      initDriver(axes[i].drv);
      axes[i].conn = axes[i].drv->test_connection();
      applyDriver(axes[i]);
    }
    if (!axes[i].active && was && axes[i].fs) axes[i].fs->forceStop();
  }
  saveSettings();
  server.send(200, "text/plain", "settings saved");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  loadSettings();

  TMCSerialA.begin(UART_BAUD, SERIAL_8N1, PIN_UART_A, PIN_UART_A);
  TMCSerialB.begin(UART_BAUD, SERIAL_8N1, PIN_UART_B, PIN_UART_B);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);                  // global enable; per-axis via toff

  pinMode(PIN_VAC_LEFT, OUTPUT);  digitalWrite(PIN_VAC_LEFT, LOW);   // vacuums OFF at boot
  pinMode(PIN_VAC_RIGHT, OUTPUT); digitalWrite(PIN_VAC_RIGHT, LOW);

  engine.init();

  for (int i = 0; i < NAX; ++i) {
    Axis& a = axes[i];
    if (!a.active) continue;                  // inactive axes stay disconnected
    connectStepper(a);
    initDriver(a.drv);
    a.conn = a.drv->test_connection();
    applyDriver(a);                           // boots with driverOn default + speed 0 => no motion
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("WiFi AP '%s' -> http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/state", HTTP_GET, handleState);
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
}

void loop() {
  server.handleClient();
}
