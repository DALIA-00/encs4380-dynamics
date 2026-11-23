/* ENCS4380 Homework 2 — "My system": micromouse drive-motor speed loop.
   Plant (one wheel): tau dv/dt + v = K u, u = PWM duty in %, v = wheel speed in m/s.
   Controller: digital PI exactly as in P3, u[k] = Kp e[k] + Ki Ts sum e[j],
   zero-order hold between samples, clamp to 0..100 % with conditional-integration
   anti-windup. The motor is stepped with forward Euler at Ts/40 (no closed form).
   Also provides the Homework 2 validation checks (window.HW2Checks) for the
   Validation page, using a continuous closed-loop Euler simulation. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const f = (n, p = 3) => Number.isFinite(n) ? Number(n.toFixed(p)).toString() : '—';
  const clip = (v, lo, hi) => Math.max(lo, Math.min(v, hi));

  /* ---------- Hardware numbers (see the table on the page) ---------- */
  const HW = { rpm: 500, gear: 30, ppr: 7, wheel: 0.043, volts: 6 };
  const COUNTS_PER_REV = HW.ppr * HW.gear * 4;                 // x4 quadrature = 840
  const M_PER_COUNT = Math.PI * HW.wheel / COUNTS_PER_REV;     // ~0.161 mm
  const DATASHEET = { K: 0.01126, tau: 0.0386 };
  const FINAL = { zeta: 0.7, ts: 0.15, r: 0.5, T: 5 };
  const SPEC = { os: 10, ts: 0.15, ess: 0.5, zLo: 0.6, zHi: 0.8 };
  let source = 'datasheet', sim = null, q = null, qIdx = 0, score = { right: 0, total: 0 };

  const read = () => ({
    K: Math.max(1e-5, +$('sysK').value || DATASHEET.K),
    tau: Math.max(1e-3, +$('sysTau').value || DATASHEET.tau),
    zeta: +$('sysZeta').value, ts: +$('sysTs').value,
    r: +$('sysR').value, Ts: +$('sysT').value / 1000
  });

  /* Pole placement for a PI on K/(tau s + 1):
     tau s^2 + (1 + K Kp) s + K Ki = 0  ->  s^2 + 2 zeta wn s + wn^2 */
  function design(p) {
    const sigma = 4 / p.ts, wn = sigma / p.zeta;
    const Kp = Math.max(0, (2 * p.tau * sigma - 1) / p.K);
    const Ki = wn * wn * p.tau / p.K;
    const zetaGains = (1 + p.K * Kp) / (2 * Math.sqrt(p.tau * p.K * Ki));
    return { Kp, Ki, wn, zetaGains, clippedKp: (2 * p.tau * sigma - 1) < 0 };
  }

  function simulate(o) {
    // o: K, tau, Kp, Ki, r, Ts, T, umin, umax, aw
    const sub = 40, h = o.Ts / sub;
    let v = 0, I = 0, peak = 0;
    const t = [0], y = [0], uT = [], u = [], raw = [];
    const steps = Math.ceil(o.T / o.Ts);
    for (let k = 0; k < steps; k++) {
      const e = o.r - v, Ic = I + o.Ts * e;
      let cmd = o.Kp * e + o.Ki * Ic;
      if (o.aw && ((cmd > o.umax && e > 0) || (cmd < o.umin && e < 0))) cmd = o.Kp * e + o.Ki * I;
      else I = Ic;
      const uk = clip(cmd, o.umin, o.umax);
      uT.push(k * o.Ts); u.push(uk); raw.push(cmd);
      for (let j = 0; j < sub; j++) {
        v += h * (o.K * uk - v) / o.tau;
        if (v > peak) peak = v;
        t.push(k * o.Ts + (j + 1) * h); y.push(v);
      }
    }
    return { t, y, uT, u, raw, peak, final: v, o };
  }
  function crossing(t, y, level) {
    for (let i = 1; i < t.length; i++) if (y[i - 1] < level && y[i] >= level)
      return t[i - 1] + (level - y[i - 1]) * (t[i] - t[i - 1]) / (y[i] - y[i - 1]);
    return NaN;
  }
  function metrics(s) {
    const r = s.o.r, band = 0.02 * r;
    let last = -1;
    for (let i = 0; i < s.y.length; i++) if (Math.abs(s.y[i] - r) > band) last = i;
    return {
      rise: crossing(s.t, s.y, 0.9 * r) - crossing(s.t, s.y, 0.1 * r),
      os: Math.max(0, (s.peak - r) / r * 100),
      ts: last >= s.y.length - 1 ? NaN : last < 0 ? 0 : s.t[last + 1],
      ess: (r - s.final) / r * 100,
      umax: Math.max(...s.u), rawMax: Math.max(...s.raw), rawMin: Math.min(...s.raw),
      satCount: s.u.filter((x, i) => Math.abs(x - s.raw[i]) > 1e-9).length
    };
  }

  /* ---------- Plots ---------- */
  const CFG = { responsive: true, displaylogo: false, displayModeBar: false };
  function pal() {
    const st = getComputedStyle(document.body);
    return { main: st.getPropertyValue('--main').trim(), dark: st.getPropertyValue('--main-dark').trim(),
      grid: st.getPropertyValue('--line').trim() || '#F3DEC6' };
  }
  function lay(x, yTitle, extra) {
    const c = pal();
    return Object.assign({ paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
      font: { family: 'Inter, sans-serif', color: '#554638', size: 11 }, margin: { l: 56, r: 14, b: 50, t: 14 },
      xaxis: { title: x, gridcolor: c.grid, zeroline: false }, yaxis: { title: yTitle, gridcolor: c.grid, zeroline: false },
      legend: { orientation: 'h', x: 0, y: 1.14 }, hovermode: 'x unified' }, extra || {});
  }
  function plot(p, d, s, alt) {
    if (!window.Plotly) return;
    const c = pal(), T = s.o.T;
    const tr = [
      { x: [0, T], y: [p.r, p.r], name: 'Setpoint r', line: { color: '#15803d', dash: 'dash', width: 2 } },
      { x: [0, T], y: [p.r * 1.02, p.r * 1.02], name: '±2 % band', line: { color: '#9ca3af', dash: 'dot', width: 1 }, hoverinfo: 'skip' },
      { x: [0, T], y: [p.r * 0.98, p.r * 0.98], showlegend: false, line: { color: '#9ca3af', dash: 'dot', width: 1 }, hoverinfo: 'skip' },
      { x: s.t, y: s.y, name: `Wheel speed (τ = ${f(p.tau * 1000, 1)} ms)`, line: { color: c.main, width: 3 } }
    ];
    alt.forEach(([k, a], i) => tr.push({ x: a.t, y: a.y, name: `If τ were ${k}× (${f(p.tau * k * 1000, 0)} ms)`,
      line: { color: i ? '#7c3aed' : '#64748b', dash: 'dash', width: 1.6 } }));
    Plotly.react('sysPlotV', tr, lay('Time (s)', 'Wheel speed v (m/s)'), CFG);
    Plotly.react('sysPlotU', [
      { x: s.uT, y: s.u, name: 'PWM command (held each Ts)', line: { color: c.main, width: 2.4, shape: 'hv' } },
      { x: [0, T], y: [100, 100], name: '100 % limit', line: { color: '#C62828', dash: 'dash' } },
      { x: [0, T], y: [0, 0], name: '0 % limit', line: { color: '#C62828', dash: 'dash' } }
    ], lay('Time (s)', 'PWM duty u (%)', { yaxis: { title: 'PWM duty u (%)', range: [-8, 112], gridcolor: c.grid, zeroline: false } }), CFG);
  }

  /* ---------- Spec table ---------- */
  function pill(ok) { return `<span class="pill ${ok ? 'pass' : 'fail'}">${ok ? 'PASS' : 'FAIL'}</span>`; }
  function specRows(p, d, m) {
    const rows = [
      ['Damping ratio ζ', `${SPEC.zLo} ≤ ζ ≤ ${SPEC.zHi}`, 'The practical range from the lecture: fast, but not oscillating. A ringing wheel speed makes the mouse wobble between walls.',
        `ζ = ${f(d.zetaGains, 3)}`, d.zetaGains >= SPEC.zLo - 1e-9 && d.zetaGains <= SPEC.zHi + 1e-9],
      ['Overshoot', `≤ ${SPEC.os} %`, 'A speed overshoot is a torque spike: the rubber wheel can slip and the encoder then over-counts the distance travelled.',
        `${f(m.os, 1)} %`, m.os <= SPEC.os],
      ['Settling time (±2 %)', `≤ ${SPEC.ts} s`, 'At 0.5 m/s one 180 mm cell takes 0.36 s; the speed must be steady before the mouse is half-way through the first cell.',
        Number.isFinite(m.ts) ? `${f(m.ts, 3)} s` : 'not settled', Number.isFinite(m.ts) && m.ts <= SPEC.ts],
      ['Steady-state error', `0 (|e| < ${SPEC.ess} %)`, 'Both wheels must reach exactly the commanded speed, otherwise the mouse curves and the cell count drifts. The integral term removes the error.',
        `${f(m.ess, 3)} %`, Math.abs(m.ess) < SPEC.ess],
      ['PWM command', '0 – 100 %, not saturated', 'The DRV8833 can only give 0–100 % duty. If the PI asks for more, the loop is no longer the linear design we computed.',
        `${f(m.rawMin, 1)} … ${f(m.rawMax, 1)} %`, m.rawMax <= 100 + 1e-9 && m.rawMin >= -1e-9]
    ];
    $('sysSpec').innerHTML = rows.map(([a, b, c, v, ok]) => `<tr><td><b>${a}</b></td><td>${b}</td><td>${c}</td><td>${v}<br>${pill(ok)}</td></tr>`).join('');
    return rows.every(r => r[4]);
  }

  /* ---------- Generated ESP32 code ---------- */
  function code(p, d) {
    const tsUs = Math.round(p.Ts * 1e6);
    return `// ENCS4380 HW2 — My system: micromouse wheel speed loop (generated)
// Team T21. Motor: JGA12-N20 6 V 500 RPM, 7 PPR Hall encoder, 43 mm wheel.
// Plant model used for the design: K = ${f(p.K, 5)} (m/s)/%, tau = ${f(p.tau, 4)} s
// Design: zeta = ${f(p.zeta, 2)}, ts = ${f(p.ts, 2)} s  ->  Kp = ${f(d.Kp, 2)}, Ki = ${f(d.Ki, 1)}
// Board: ESP32 (Arduino-ESP32 core). Driver: DRV8833, motor supply 6 V.
// Pins are placeholders: copy them from robot_config.h once M3/M4 are done.
#include <Arduino.h>

const int ENC_A = 34, ENC_B = 35;       // encoder channels (TODO: robot_config.h)
const int IN1 = 25, IN2 = 26;           // DRV8833 AIN1 / AIN2 (TODO: robot_config.h)

const uint32_t TS_US = ${tsUs};               // sampling time Ts = ${f(p.Ts * 1000, 1)} ms
const float Ts = ${p.Ts.toFixed(4)}f;          // seconds
const float Kp = ${d.Kp.toFixed(3)}f;          // % PWM per (m/s)
const float Ki = ${d.Ki.toFixed(2)}f;          // % PWM per (m/s * s)
const float SETPOINT = ${p.r.toFixed(3)}f;     // m/s, cruise speed step
const float UMIN = 0.0f, UMAX = 100.0f;  // PWM duty limits (%)
const bool ANTI_WINDUP = true;

const float COUNTS_PER_REV = ${COUNTS_PER_REV}.0f;  // 7 PPR x 30 gear x 4 (quadrature)
const float WHEEL_D = ${HW.wheel}f;              // m
const float M_PER_COUNT = PI * WHEEL_D / COUNTS_PER_REV;

volatile long encCount = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t encPrev = 0;

// x4 quadrature decoding: called on every edge of A and B.
// If the speed comes out negative, swap ENC_A and ENC_B.
void IRAM_ATTR encISR() {
  static const int8_t table[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  uint8_t cur = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  portENTER_CRITICAL_ISR(&encMux);
  encCount += table[(encPrev << 2) | cur];
  encPrev = cur;
  portEXIT_CRITICAL_ISR(&encMux);
}

void setPWM(float duty) {               // forward only, fast decay
  analogWrite(IN1, (int)(duty * 2.55f + 0.5f));  // 8-bit PWM, 0..255
  analogWrite(IN2, 0);
}

float integral = 0.0f;
long lastCount = 0;
uint32_t lastTick, startTime;

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  encPrev = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);
  setPWM(0);
  delay(2000);                          // time to put the mouse down
  Serial.println("t,v,u");
  startTime = lastTick = micros();
}

void loop() {
  uint32_t now = micros();
  if (now - lastTick < TS_US) return;   // run exactly once every Ts
  lastTick += TS_US;
  float t = (lastTick - startTime) * 1e-6f;

  portENTER_CRITICAL(&encMux);
  long c = encCount;
  portEXIT_CRITICAL(&encMux);
  float v = (c - lastCount) * M_PER_COUNT / Ts;   // measured wheel speed, m/s
  lastCount = c;

  float r = (t < 1.5f) ? SETPOINT : 0.0f;         // 1.5 s step test, then stop
  float e = r - v;
  float candidate = integral + e * Ts;
  float raw = Kp * e + Ki * candidate;            // u[k] = Kp e[k] + Ki Ts sum e[j]
  if (ANTI_WINDUP && ((raw > UMAX && e > 0) || (raw < UMIN && e < 0))) {
    raw = Kp * e + Ki * integral;                 // freeze the integral while clamped
  } else {
    integral = candidate;
  }
  float u = constrain(raw, UMIN, UMAX);
  if (r == 0.0f) { u = 0; integral = 0; }
  setPWM(u);                                      // held until the next sample (ZOH)

  if (t < 2.0f) {                                 // log for comparison with the page
    Serial.print(t, 3); Serial.print(',');
    Serial.print(v, 4); Serial.print(',');
    Serial.println(u, 1);
  }
}`;
  }

  /* ---------- Predict, then check ---------- */
  const QS = [
    p => { const d = design(p); const a = clip(d.Kp * p.r + d.Ki * p.Ts * p.r, 0, 100);
      return { text: `At the first sample the wheel is still (e = r = ${f(p.r, 2)} m/s). With Kp = ${f(d.Kp, 1)}, Ki = ${f(d.Ki, 0)} and Ts = ${f(p.Ts * 1000, 0)} ms, what PWM duty does the controller command?`,
        unit: '%', ans: a, tol: Math.max(1, 0.05 * a), why: `u[0] = Kp·r + Ki·Ts·r = ${f(d.Kp * p.r, 2)} + ${f(d.Ki * p.Ts * p.r, 2)}, clamped to 0–100 %.` }; },
    p => { const a = p.r / p.K;
      return { text: `Once the wheel turns steadily at r = ${f(p.r, 2)} m/s, what PWM duty is needed to hold it there? (K = ${f(p.K, 5)} (m/s)/%)`,
        unit: '%', ans: a, tol: Math.max(1, 0.05 * a), why: `In steady state v = K·u, so u = r/K. The error is zero, so this PWM comes entirely from the integral term.` }; },
    p => { const a = p.r * p.Ts / M_PER_COUNT;
      return { text: `At ${f(p.r, 2)} m/s, how many encoder counts arrive in one sample of Ts = ${f(p.Ts * 1000, 0)} ms? (840 counts per wheel turn, 43 mm wheel)`,
        unit: 'counts', ans: a, tol: Math.max(1, 0.08 * a), why: `Counts = v·Ts / (π·0.043/840) = ${f(p.r * p.Ts * 1000, 2)} mm / ${f(M_PER_COUNT * 1000, 4)} mm. Few counts per sample means a coarse speed reading, which is why Ts cannot be made tiny.` }; },
    p => { const m = metrics(sim);
      return { text: `Look at the design settings, then predict: what overshoot (%) will the simulated wheel speed have?`,
        unit: '%', ans: m.os, tol: 3, why: `The ζ-formula alone predicts ${f(100 * Math.exp(-Math.PI * p.zeta / Math.sqrt(Math.max(1e-9, 1 - p.zeta * p.zeta))), 1)} %, but the PI zero adds overshoot, and PWM saturation can reduce it.` }; }
  ];
  function newQ() { qIdx = Math.floor(Math.random() * QS.length); q = QS[qIdx](read()); showQ(); }
  function showQ() {
    $('sysQ').textContent = q.text; $('sysUnit').textContent = q.unit; $('sysAns').value = '';
    $('sysFb').className = 'fb'; $('sysFb').textContent = '';
    $('sysScore').textContent = `Session score: ${score.right} / ${score.total}`;
  }
  function check() {
    const g = parseFloat($('sysAns').value); if (!Number.isFinite(g)) return;
    q = QS[qIdx](read());          // answer from the settings shown right now
    $('sysQ').textContent = q.text;
    const ok = Math.abs(g - q.ans) <= q.tol;
    score.total++; if (ok) score.right++;
    $('sysFb').className = 'fb ' + (ok ? 'ok' : 'no');
    $('sysFb').textContent = `${ok ? 'Correct!' : 'Not quite.'} Answer: ${f(q.ans, 2)} ${q.unit}. ${q.why}`;
    $('sysScore').textContent = `Session score: ${score.right} / ${score.total}`;
  }

  /* ---------- Step-test fit ---------- */
  function fit() {
    const step = +$('sysStep').value, fb = $('sysFitFb');
    const pts = $('sysCsv').value.split(/\n/).map(l => l.trim().split(/[\s,;]+/).map(Number))
      .filter(a => a.length >= 2 && a.every(Number.isFinite));
    if (pts.length < 8 || !(step > 0)) { fb.className = 'fb no'; fb.textContent = 'Need at least 8 lines of "t, speed" and a PWM step above 0.'; return; }
    const t0 = pts[0][0], tail = pts.slice(Math.floor(pts.length * 0.8));
    const vf = tail.reduce((s, a) => s + a[1], 0) / tail.length;
    const t63 = crossing(pts.map(a => a[0] - t0), pts.map(a => a[1]), 0.632 * vf);
    if (!(vf > 0) || !Number.isFinite(t63)) { fb.className = 'fb no'; fb.textContent = 'Could not find the 63.2 % point. Check that speed rises from 0 and the log is long enough to settle.'; return; }
    $('sysK').value = (vf / step).toFixed(5); $('sysTau').value = t63.toFixed(4); source = 'measured';
    fb.className = 'fb ok';
    fb.textContent = `Final speed ${f(vf, 3)} m/s → K = ${f(vf / step, 5)} (m/s)/%; 63.2 % reached at τ = ${f(t63 * 1000, 1)} ms. The design below now uses the measured plant.`;
    update();
  }

  /* ---------- Main update ---------- */
  function update() {
    const p = read(), d = design(p);
    $('sysZetaV').textContent = f(p.zeta, 2); $('sysTsV').textContent = f(p.ts, 2) + ' s';
    $('sysRV').textContent = f(p.r, 2) + ' m/s'; $('sysTV').textContent = f(p.Ts * 1000, 0) + ' ms';
    const same = Math.abs(p.K - DATASHEET.K) < 1e-9 && Math.abs(p.tau - DATASHEET.tau) < 1e-9;
    if (same) source = 'datasheet';
    $('sysSource').innerHTML = source === 'measured' ? '<b>Source:</b> measured step test (fitted above).'
      : same ? '<b>Source:</b> datasheet estimate. K from the listing (500 RPM at 6 V, 43 mm wheel); τ calculated, not yet measured.'
      : '<b>Source:</b> typed in by hand.';
    const T = Math.max(0.4, 3 * p.ts, 8 * p.tau);
    const base = { K: p.K, tau: p.tau, Kp: d.Kp, Ki: d.Ki, r: p.r, Ts: p.Ts, T, umin: 0, umax: 100, aw: true };
    sim = simulate(base);
    const alt = [0.5, 1.5].map(k => [k, simulate(Object.assign({}, base, { tau: p.tau * k }))]);
    const m = metrics(sim), altM = alt.map(([k, a]) => [k, metrics(a)]);
    $('sysGains').innerHTML = [['Kp', f(d.Kp, 1) + ' %/(m/s)'], ['Ki', f(d.Ki, 0) + ' %/(m/s·s)'], ['ωₙ', f(d.wn, 1) + ' rad/s'], ['ζ from gains', f(d.zetaGains, 3)]]
      .map(([k, v]) => `<div class="stat"><div class="k">${k}</div><div class="v">${v}</div></div>`).join('');
    const allPass = specRows(p, d, m);
    $('sysMetrics').innerHTML = [
      ['Rise time (10–90 %)', f(m.rise * 1000, 1) + ' ms'], ['Overshoot', f(m.os, 1) + ' %'],
      ['Settling time (±2 %)', Number.isFinite(m.ts) ? f(m.ts * 1000, 0) + ' ms' : 'not settled'], ['Steady-state error', f(m.ess, 3) + ' %'],
      ['Peak PWM', f(m.umax, 1) + ' %'], ['Holding PWM r/K', f(p.r / p.K, 1) + ' %'],
      ['Counts per sample', f(p.r * p.Ts / M_PER_COUNT, 1)],
      ['If τ ×0.5 / ×1.5', altM.map(([k, a]) => `${f(a.os, 1)} % OS`).join(' / ')],
      ['Whole spec', allPass ? '✅ PASS' : '❌ not met']
    ].map(([k, v]) => `<div class="stat"><div class="k">${k}</div><div class="v">${v}</div></div>`).join('');
    plot(p, d, sim, alt);
    $('sysCode').textContent = code(p, d);
    if (!q) newQ();
  }
  function final() {
    $('sysZeta').value = FINAL.zeta; $('sysTs').value = FINAL.ts; $('sysR').value = FINAL.r; $('sysT').value = FINAL.T;
    update();
  }

  /* ---------- Homework 2 validation checks (Validation page) ---------- */
  function contLoop(K, tau, Kp, Ki, T, dt) {
    // Continuous PI, forward Euler, no limits: y' = (K u - y)/tau, u = Kp e + Ki ∫e
    let y = 0, I = 0, peak = 0; const t = [0], ys = [0];
    for (let i = 1; i * dt <= T + 1e-12; i++) {
      const e = 1 - y; const u = Kp * e + Ki * I;
      I += dt * e; y += dt * (K * u - y) / tau;
      if (y > peak) peak = y;
      t.push(i * dt); ys.push(y);
    }
    return { t, y: ys, final: y, peak };
  }
  window.HW2Checks = function () {
    const K = 1, tau = 1, dt = 1e-4;
    const a = contLoop(K, tau, 9, 0, 2, dt);
    const tcl = crossing(a.t, a.y, 0.632 * a.final), ess = (1 - a.final) * 100;
    const wn = Math.sqrt(K * 1 / tau), zeta = (1 + K * 1) / (2 * Math.sqrt(tau * K * 1));
    const b = contLoop(K, tau, 1, 1, 10, dt);
    const y1 = b.y[Math.round(1 / dt)], os = Math.max(0, (b.peak - 1) * 100);
    const sets = [[0.5, 0.5], [1, 1], [2, 5], [5, 20], [0.2, 3]];
    const errs = sets.map(([kp, ki]) => Math.abs(1 - contLoop(K, tau, kp, ki, 40, 1e-3).final) * 100);
    return [
      ['HW2 · P only, Kp = 9 (K = 1, τ = 1 s)', 'τ<sub>cl</sub> = τ/(1+KKp) = 0.1 s; e<sub>ss</sub> = 1/(1+KKp) = 10 %',
        `τ<sub>cl</sub> = ${f(tcl, 4)} s; e<sub>ss</sub> = ${f(ess, 3)} %`, Math.abs(tcl - 0.1) < 0.002 && Math.abs(ess - 10) < 0.05],
      ['HW2 · PI, Kp = 1, Ki = 1', 'ωₙ = √(KKi/τ) = 1 rad/s; ζ = (1+KKp)/(2√(τKKi)) = 1',
        `ωₙ = ${f(wn, 4)}, ζ = ${f(zeta, 4)}; simulated y(1 s) = ${f(y1, 4)} (1 − e⁻¹ = 0.6321), overshoot ${f(os, 3)} %`,
        Math.abs(wn - 1) < 1e-9 && Math.abs(zeta - 1) < 1e-9 && Math.abs(y1 - (1 - Math.exp(-1))) < 0.002 && os < 0.01],
      ['HW2 · any PI: zero steady-state error', 'e<sub>ss</sub> = 0 for a step',
        sets.map(([kp, ki], i) => `(${kp}, ${ki}): ${f(errs[i], 4)} %`).join('; '), errs.every(e => e < 0.01)]
    ];
  };

  /* ---------- Events ---------- */
  ['sysZeta', 'sysTs', 'sysR', 'sysT'].forEach(id => $(id).addEventListener('input', update));
  ['sysK', 'sysTau'].forEach(id => $(id).addEventListener('input', () => { source = 'manual'; update(); }));
  $('sysDatasheet').addEventListener('click', () => { $('sysK').value = DATASHEET.K; $('sysTau').value = DATASHEET.tau; source = 'datasheet'; update(); });
  $('sysFinal').addEventListener('click', final);
  $('sysFit').addEventListener('click', fit);
  $('sysNew').addEventListener('click', newQ);
  $('sysCheck').addEventListener('click', check);
  $('sysAns').addEventListener('keydown', e => { if (e.key === 'Enter') check(); });
  $('sysCopy').addEventListener('click', async () => {
    try { await navigator.clipboard.writeText($('sysCode').textContent); $('sysCopy').textContent = 'Copied!'; }
    catch (e) { const r = document.createRange(); r.selectNodeContents($('sysCode')); const s = getSelection(); s.removeAllRanges(); s.addRange(r); $('sysCopy').textContent = 'Press Ctrl+C'; }
    setTimeout(() => { $('sysCopy').textContent = 'Copy code'; }, 1800);
  });

  window.SysLoop = { update, simulate, design, metrics };
  update();
  if (document.getElementById('val') && document.getElementById('val').classList.contains('show') && window.renderValAll) window.renderValAll();
})();
