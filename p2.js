/* ENCS4380 HW2 — P2 A slow sensor in the loop (Dalia Abu Baker)

   The loop:   r → Σ → PID → limit ±Umax → plant → y (true output)
                   ↑                               │
                   └──── ym ←── sensor 1/(τs·s+1) ←┘

   The controller never sees y. It only sees the sensor reading ym, so
   the error is e = r − ym. A slow sensor (large τs) reports y late, the
   controller keeps pushing after y has already reached the setpoint, and
   the true output overshoots more and more until the loop oscillates.

   Simulation: forward Euler, all states stepped together in time
   (plant, sensor, integral). No closed-form response is used.
   Everything runs in the browser. P1's file (p1.js) is not changed.
*/
(() => {
  'use strict';
  const id = x => document.getElementById(x);
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const fmt = (x, digits = 3) => Number.isFinite(x) ? Number(x.toFixed(digits)).toString() : '—';
  const sliderIds = ['p2K', 'p2tau', 'p2zeta', 'p2wn', 'p2taus', 'p2kp', 'p2ki', 'p2kd', 'p2setpoint', 'p2limit', 'p2time'];
  const TAUS_MAX = 3;            // largest sensor τs on the slider and in the sweep plot
  const defaults = {kind: 'first', K: 1, tau: 1, zeta: .5, wn: 2, taus: .3, kp: 4, ki: 8, kd: 0, setpoint: 1, limit: 50, time: 20};
  // Second-order plant: critically damped, ωn = 1 rad/s. With these gains Routh gives τs,crit = 1.5 s.
  const SECOND_DEFAULTS = {...defaults, kind: 'second', zeta: 1, wn: 1, taus: .3, kp: 3, ki: 2, kd: 0, time: 30};
  let plantKind = 'first', result = null, sweepTimer = null, animFrame = null;
  let question = null, correct = 0, answered = 0;

  function read() {
    return {
      kind: plantKind, K: +id('p2K').value, tau: +id('p2tau').value,
      zeta: +id('p2zeta').value, wn: +id('p2wn').value, taus: +id('p2taus').value,
      kp: +id('p2kp').value, ki: +id('p2ki').value, kd: +id('p2kd').value,
      setpoint: +id('p2setpoint').value, limit: +id('p2limit').value, time: +id('p2time').value
    };
  }

  /* ---------- Simulation (forward Euler) ----------
     States:  y  (true plant output), v = dy/dt (second-order plant only),
              ym (sensor reading), I = ∫e dt (integral of the error).
     Sensor:  τs·dym/dt = y − ym       (τs = 0 means an ideal sensor: ym = y)
     PID:     u = Kp·e + Ki·I − Kd·dym/dt,  with e = r − ym
              The D term acts on the sensor reading (what the controller can
              measure), not on e, so a setpoint step gives no derivative kick.
     Limit:   u is clipped to ±Umax. The integral stops growing while the
              actuator is saturated in the same direction (anti-windup), as in P1.
     Plant:   first order   τ·dy/dt = K·u − y              (HW1 Module 1)
              second order  y'' = K·ωn²·u − 2ζωn·y' − ωn²·y (HW1 Module 3)  */
  function stepSize(p) {
    // Rough fastest rate in the loop; Euler needs dt well below 1/rate.
    const s = p.taus > 0 ? p.taus : Infinity;
    const plantRate = p.kind === 'first'
      ? (1 + p.K * p.kp) / p.tau + p.K * p.kd / (p.tau * Math.min(s, 1e3))
      : 2 * p.zeta * p.wn + p.wn * Math.sqrt(1 + p.K * p.kp) + p.K * p.wn * p.wn * p.kd / Math.min(s, 1e3);
    const rate = plantRate + (p.taus > 0 ? 1 / p.taus : 0) + Math.sqrt(Math.abs(p.K * p.ki));
    return clamp(.015 / rate, 5e-5, 1e-3);
  }

  function simulate(p, keep = 1500) {
    const dt0 = stepSize(p), steps = Math.ceil(p.time / dt0), dt = p.time / steps;
    const stride = Math.max(1, Math.floor(steps / keep));
    const t = [], yOut = [], ymOut = [], uOut = [];
    let y = 0, v = 0, ym = 0, I = 0, dym = 0, diverged = false, saturated = 0;
    for (let i = 0; i <= steps; i++) {
      // 1) The controller reads only the sensor.
      if (p.taus > 0) dym = (y - ym) / p.taus; else { ym = y; dym = p.kind === 'first' ? dym : v; }
      const e = p.setpoint - ym;
      // 2) PID with conditional-integration anti-windup.
      let Itrial = I + e * dt;
      let uReq = p.kp * e + p.ki * Itrial - p.kd * dym;
      if ((uReq > p.limit && e > 0) || (uReq < -p.limit && e < 0)) Itrial = I;
      uReq = p.kp * e + p.ki * Itrial - p.kd * dym;
      const u = clamp(uReq, -p.limit, p.limit);
      if (Math.abs(uReq) > p.limit) saturated++;
      if (i % stride === 0 || i === steps) { t.push(i * dt); yOut.push(y); ymOut.push(ym); uOut.push(u); }
      if (i === steps) break;
      // 3) Advance the plant and the sensor by one Euler step.
      let dy;
      if (p.kind === 'first') {
        dy = (p.K * u - y) / p.tau;
        y += dt * dy;
        if (p.taus === 0) dym = dy;
      } else {
        const acc = p.K * p.wn * p.wn * u - 2 * p.zeta * p.wn * v - p.wn * p.wn * y;
        y += dt * v; v += dt * acc;
      }
      if (p.taus > 0) ym += dt * dym;
      I = Itrial;
      if (!Number.isFinite(y) || Math.abs(y) > 1e6) { diverged = true; break; }
    }
    return {t, y: yOut, ym: ymOut, u: uOut, diverged, saturatedShare: saturated / (steps + 1), params: p};
  }

  /* ---------- Linear analysis (Routh–Hurwitz) ----------
     With the sensor in the feedback path the characteristic equation is
         s·D(s)·(τs·s + 1) + K·N(s)·(Kd·s² + Kp·s + Ki) = 0
     First-order plant, D = τs + 1:
         τ·τs·s³ + (τ + τs + K·Kd)·s² + (1 + K·Kp)·s + K·Ki = 0
     and Routh gives the critical sensor time constant directly:
         τs,crit = (τ + K·Kd)(1 + K·Kp) / (τ·K·Ki − (1 + K·Kp)).
     For the second-order plant (4th-order equation) τs,crit is found by bisection.
     Saturation is nonlinear, so these are predictions for the unsaturated loop. */
  const polyMul = (a, b) => { const r = Array(a.length + b.length - 1).fill(0); a.forEach((x, i) => b.forEach((y, j) => { r[i + j] += x * y; })); return r; };
  const polyAdd = (a, b) => { const n = Math.max(a.length, b.length), r = Array(n).fill(0); a.forEach((x, i) => { r[n - a.length + i] += x; }); b.forEach((x, i) => { r[n - b.length + i] += x; }); return r; };
  function charPoly(p, taus) {
    const D = p.kind === 'first' ? [p.tau, 1] : [1, 2 * p.zeta * p.wn, p.wn * p.wn];
    const N = p.kind === 'first' ? [p.K] : [p.K * p.wn * p.wn];
    let c = polyAdd(polyMul(polyMul([1, 0], D), [taus, 1]), polyMul(N, [p.kd, p.kp, p.ki]));
    while (c.length > 1 && Math.abs(c[0]) < 1e-12) c = c.slice(1);              // τs = 0 lowers the order
    while (c.length > 1 && Math.abs(c[c.length - 1]) < 1e-12) c = c.slice(0, -1); // Ki = 0: a factor s cancels
    return c;
  }
  function routhStable(c) {
    if (c.length <= 1) return true;
    if (c.some(x => !(x > 0))) return false;
    let rows = [c.filter((_, i) => i % 2 === 0), c.filter((_, i) => i % 2 === 1)];
    for (let k = 2; k < c.length; k++) {
      const a = rows[k - 2], b = rows[k - 1], next = [];
      if (!(b[0] > 0)) return false;
      for (let j = 0; j < a.length - 1; j++) next.push((b[0] * (a[j + 1] || 0) - a[0] * (b[j + 1] || 0)) / b[0]);
      if (!next.length) next.push(0);
      rows.push(next);
    }
    return rows.every(r => r[0] > 0);
  }
  const stableAt = (p, taus) => routhStable(charPoly(p, taus));
  function criticalTaus(p) {
    if (!stableAt(p, 0)) return 0;                     // unstable even with an ideal sensor
    if (p.kind === 'first') {
      const den = p.tau * p.K * p.ki - (1 + p.K * p.kp);
      return den > 0 ? (p.tau + p.K * p.kd) * (1 + p.K * p.kp) / den : Infinity;
    }
    let lo = 0, hi = 0.01;
    while (stableAt(p, hi)) { lo = hi; hi *= 2; if (hi > 1e3) return Infinity; }
    for (let k = 0; k < 50; k++) { const m = (lo + hi) / 2; if (stableAt(p, m)) lo = m; else hi = m; }
    return (lo + hi) / 2;
  }

  /* ---------- Metrics read off the simulated curves ---------- */
  function finalTarget(p) {
    // Steady state with an integral term is r; with no integral term the plant
    // DC gain K and Kp give r·K·Kp/(1 + K·Kp). The sensor (DC gain 1) changes neither.
    return p.ki > 0 ? p.setpoint : p.setpoint * p.K * p.kp / (1 + p.K * p.kp);
  }
  function overshootOf(arr, target) {
    return target > 0 ? Math.max(0, (Math.max(...arr) - target) / target * 100) : NaN;
  }
  function settlingOf(t, arr, target) {
    const band = .02 * Math.abs(target);
    let last = -1;
    for (let i = 0; i < arr.length; i++) if (Math.abs(arr[i] - target) > band) last = i;
    return last >= arr.length - 1 ? NaN : last < 0 ? 0 : t[last + 1];
  }
  // Compare the swing of y in the third and fourth quarters of the run:
  // shrinking → the oscillation dies out, about equal → sustained, growing → unstable.
  function loopStatus(s, target) {
    if (s.diverged) return {key: 'growing', text: 'Unstable — grows without limit'};
    const n = s.y.length, swing = (a, b) => { const part = s.y.slice(a, b); return Math.max(...part) - Math.min(...part); };
    const s3 = swing(Math.floor(n / 2), Math.floor(3 * n / 4)), s4 = swing(Math.floor(3 * n / 4), n);
    const small = .01 * Math.max(Math.abs(target), 1e-6);
    if (s4 < small) return {key: 'settles', text: 'Settles'};
    const ratio = s4 / Math.max(s3, 1e-12);
    if (ratio > 1.05) return {key: 'growing', text: 'Growing oscillation (unstable)'};
    if (ratio > .95) return {key: 'sustained', text: 'Sustained oscillation'};
    return {key: 'decaying', text: 'Oscillation still dying out'};
  }
  function metrics(s) {
    const p = s.params, target = finalTarget(p);
    const status = loopStatus(s, target);
    const ok = status.key === 'settles' || status.key === 'decaying';
    return {
      target, status,
      // Overshoot only means something for a loop that settles.
      trueOS: ok ? overshootOf(s.y, target) : NaN,
      sensorOS: ok ? overshootOf(s.ym, target) : NaN,
      settle: ok ? settlingOf(s.t, s.y, target) : NaN,
      err: ok ? (p.setpoint - s.y.at(-1)) / p.setpoint * 100 : NaN,
      crit: criticalTaus(p)
    };
  }

  /* ---------- Drawing ---------- */
  function labels(p) {
    sliderIds.forEach(name => { id(name + 'V').textContent = id(name).value; });
    const first = p.kind === 'first';
    id('p2firstControls').style.display = first ? '' : 'none';
    id('p2secondControls').style.display = first ? 'none' : '';
    id('p2plantLabel').textContent = first ? 'First-order plant' : 'Second-order plant';
    id('p2equation').textContent = first ? 'G(s) = K / (τs + 1)' : 'G(s) = Kωₙ² / (s² + 2ζωₙs + ωₙ²)';
    id('p2diagramPlant').textContent = first ? 'K / (τs+1)' : 'Kωₙ² / (s²+2ζωₙs+ωₙ²)';
    id('p2diagramSensor').textContent = 'Sensor 1 / (' + fmt(p.taus, 2) + '·s + 1)';
    document.querySelectorAll('#p2plantBtns button').forEach(b => b.classList.toggle('on', b.dataset.kind === p.kind));
  }
  const colors = () => {
    const css = getComputedStyle(document.body);
    return {main: css.getPropertyValue('--main').trim() || '#E24C00', sensor: '#1565C0', set: '#2E7D32', bad: '#C62828', muted: '#7A5A44'};
  };
  const layout = (xTitle, yTitle, extra = {}) => ({
    paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
    margin: {l: 55, r: 18, t: 22, b: 55}, font: {family: 'Inter, sans-serif', color: '#7A5A44', size: 12},
    xaxis: {title: xTitle, gridcolor: '#F3DEC6', zeroline: false},
    yaxis: {title: yTitle, gridcolor: '#F3DEC6', zeroline: false},
    legend: {orientation: 'h', y: 1.15, x: 0}, hovermode: 'x unified', ...extra
  });
  const cfg = {responsive: true, displayModeBar: false};

  function plotResponse(s) {
    if (!window.Plotly) return;
    const C = colors(), p = s.params;
    window.Plotly.react('p2response', [
      {x: s.t, y: s.t.map(() => p.setpoint), name: 'Setpoint r', mode: 'lines', line: {color: C.set, width: 2, dash: 'dash'}},
      {x: s.t, y: s.y, name: 'True output y', mode: 'lines', line: {color: C.main, width: 3}},
      {x: s.t, y: s.ym, name: 'Sensor reading ym', mode: 'lines', line: {color: C.sensor, width: 2.5, dash: 'dot'}}
    ], layout('Time (s)', 'Output'), cfg);
    window.Plotly.react('p2control', [
      {x: s.t, y: s.u, name: 'Control u(t)', mode: 'lines', line: {color: C.main, width: 2.2}},
      {x: [0, p.time], y: [p.limit, p.limit], name: '±Umax', mode: 'lines', line: {color: C.bad, width: 1.5, dash: 'dot'}},
      {x: [0, p.time], y: [-p.limit, -p.limit], showlegend: false, mode: 'lines', line: {color: C.bad, width: 1.5, dash: 'dot'}, hoverinfo: 'skip'}
    ], layout('Time (s)', 'Control u'), cfg);
  }

  // Overshoot of the true output and of the sensor reading for τs from 0 to TAUS_MAX,
  // with the current gains. This is the "overshoot grows until the loop oscillates" picture.
  function plotSweep() {
    if (!window.Plotly || !id('p2sweep')) return;
    const p = read(), C = colors(), target = finalTarget(p), crit = criticalTaus(p);
    const xs = [], trueOS = [], sensOS = [];
    for (let k = 0; k <= 40; k++) {
      const taus = TAUS_MAX * k / 40;
      const s = simulate({...p, taus}, 400);
      const st = loopStatus(s, target).key;
      xs.push(taus);
      const unstable = s.diverged || st === 'growing' || st === 'sustained';
      trueOS.push(unstable ? null : overshootOf(s.y, target));
      sensOS.push(unstable ? null : overshootOf(s.ym, target));
    }
    const shapes = [{type: 'line', x0: p.taus, x1: p.taus, yref: 'paper', y0: 0, y1: 1, line: {color: C.muted, width: 2, dash: 'dash'}}];
    const annotations = [{x: p.taus, yref: 'paper', y: 1, text: 'now', showarrow: false, yanchor: 'bottom', font: {size: 11}}];
    // Shade every τs range that Routh–Hurwitz calls unstable (for the 4th-order
    // case the unstable range does not always run all the way to the right edge).
    let start = null, labelled = false;
    for (let k = 0; k <= 300; k++) {
      const taus = TAUS_MAX * k / 300, bad = !stableAt(p, taus);
      if (bad && start === null) start = taus;
      if ((!bad || k === 300) && start !== null) {
        const end = bad ? TAUS_MAX : taus;
        shapes.push({type: 'rect', x0: start, x1: end, yref: 'paper', y0: 0, y1: 1, fillcolor: 'rgba(198,40,40,.10)', line: {width: 0}});
        if (!labelled && (labelled = true))
          annotations.push({x: (start + end) / 2, yref: 'paper', y: .5, text: 'Loop oscillates<br>(τs > ' + fmt(crit, 2) + ' s)', showarrow: false, font: {color: C.bad, size: 12}});
        start = null;
      }
    }
    window.Plotly.react('p2sweep', [
      {x: xs, y: trueOS, name: 'True overshoot', mode: 'lines+markers', marker: {size: 4}, line: {color: C.main, width: 3}},
      {x: xs, y: sensOS, name: 'Overshoot the sensor reports', mode: 'lines+markers', marker: {size: 4}, line: {color: C.sensor, width: 2, dash: 'dot'}}
    ], layout('Sensor time constant τs (s)', 'Overshoot (%)', {shapes, annotations, xaxis: {title: 'Sensor time constant τs (s)', gridcolor: '#F3DEC6', zeroline: false, range: [0, TAUS_MAX]}}), cfg);
  }
  function scheduleSweep() {
    clearTimeout(sweepTimer);
    sweepTimer = setTimeout(plotSweep, animFrame ? 400 : 120);
  }

  function renderStats(s) {
    const m = metrics(s), show = (v, unit, d = 1) => Number.isFinite(v) ? fmt(v, d) + unit : '—';
    const critText = m.crit === 0 ? 'Unstable even with an ideal sensor' : Number.isFinite(m.crit) ? fmt(m.crit, 3) + ' s' : 'Never (no limit)';
    const rows = [
      ['True overshoot', show(m.trueOS, ' %')],
      ['Overshoot the sensor reports', show(m.sensorOS, ' %')],
      ['Hidden overshoot (true − sensor)', show(m.trueOS - m.sensorOS, ' %')],
      ['True settling time (±2%)', Number.isFinite(m.settle) ? fmt(m.settle, 2) + ' s' : 'Not settled'],
      ['Steady-state error', show(m.err, ' %', 2)],
      ['Loop behaviour', m.status.text],
      ['Oscillation starts at τs (Routh)', critText]
    ];
    id('p2metrics').innerHTML = rows.map(([k, v]) => '<div class="stat"><div class="k">' + k + '</div><div class="v">' + v + '</div></div>').join('');
    const badge = id('p2badge');
    badge.textContent = m.status.text;
    badge.className = 'p2-badge ' + m.status.key;
    let note = 'Overshoot is measured against the expected final value ' + fmt(m.target) + '. ';
    if (s.saturatedShare > .02) note += 'The actuator is at its limit ' + fmt(100 * s.saturatedShare, 0) + '% of the time, so the response is slower than the linear (Routh) prediction. ';
    if (m.status.key === 'decaying') note += 'Make the simulation time longer to see it settle.';
    id('p2metricsNote').textContent = note;
    return m;
  }

  function update() {
    const p = read(); labels(p);
    result = simulate(p);
    renderStats(result);
    plotResponse(result);
    scheduleSweep();
  }
  let frame = null;
  const scheduleUpdate = () => { if (frame !== null) cancelAnimationFrame(frame); frame = requestAnimationFrame(() => { frame = null; update(); }); };

  /* ---------- Animate τs from 0 up past the oscillation point ---------- */
  function animate() {
    if (animFrame) { stopAnimation(); return; }
    const start = performance.now(), duration = 7000;
    id('p2animate').textContent = '■ Stop';
    id('p2taus').value = 0;
    const tick = now => {
      const f = Math.min(1, (now - start) / duration);
      id('p2taus').value = (Math.round(f * TAUS_MAX * 100) / 100).toFixed(2);
      update();
      if (f < 1) animFrame = requestAnimationFrame(tick); else stopAnimation();
    };
    animFrame = requestAnimationFrame(tick);
  }
  function stopAnimation() {
    if (animFrame) cancelAnimationFrame(animFrame);
    animFrame = null;
    id('p2animate').textContent = '▶ Slow the sensor down';
    scheduleSweep();
  }

  /* ---------- Predict, then check ---------- */
  const QUESTIONS = [
    {
      ask: p => 'With the current gains, what final value will the TRUE output y settle at? (The sensor is slow — does that change it?)',
      unit: '', answer: p => finalTarget(p), tol: a => Math.max(.01, .02 * Math.abs(a)),
      explain: (p, a) => (p.ki > 0 ? 'With an integral term the error r − ym is driven to zero, and the sensor\'s DC gain is 1, so y → r = ' : 'With no integral term, y∞ = r·K·Kp/(1 + K·Kp) = ') + fmt(a, 3) + '. The sensor only delays the reading; it does not change the steady state.'
    },
    {
      ask: p => 'Without moving τs: at what sensor time constant τs does this loop start to oscillate without dying out?',
      unit: 's', answer: p => criticalTaus(p), usable: p => { const c = criticalTaus(p); return Number.isFinite(c) && c > 0; },
      tol: a => Math.max(.03, .08 * a),
      explain: (p, a) => p.kind === 'first'
        ? 'Routh on τ·τs·s³ + (τ + τs + K·Kd)s² + (1 + K·Kp)s + K·Ki = 0 gives τs,crit = (τ + K·Kd)(1 + K·Kp) / (τ·K·Ki − (1 + K·Kp)) = ' + fmt(a, 3) + ' s. Use the animation to watch it happen.'
        : 'Routh–Hurwitz on the 4th-order characteristic equation gives τs,crit ≈ ' + fmt(a, 3) + ' s. Use the animation to watch it happen.'
    },
    {
      ask: p => 'Predict the overshoot (%) of the TRUE output y for the current settings. Will it be bigger or smaller than what the sensor reports?',
      unit: '%', answer: p => metrics(simulate(p)).trueOS, usable: p => { const m = metrics(simulate(p)); return Number.isFinite(m.trueOS) && (m.status.key === 'settles' || m.status.key === 'decaying'); },
      tol: a => Math.max(3, .15 * a),
      explain: (p, a) => { const m = metrics(simulate(p)); return 'The simulation gives ' + fmt(a, 1) + ' % for y, but the sensor reports only ' + fmt(m.sensorOS, 1) + ' %. The sensor lags and smooths the peak, so the real overshoot is hidden.'; }
    }
  ];
  function newQuestion() {
    const p = read(), usable = QUESTIONS.filter(q => !q.usable || q.usable(p));
    const pool = usable.filter(q => q !== question);
    question = (pool.length ? pool : usable)[Math.floor(Math.random() * (pool.length || usable.length))];
    id('p2qText').textContent = question.ask(p);
    id('p2qUnit').textContent = question.unit;
    id('p2qAns').value = '';
    id('p2qFb').style.display = 'none';
  }
  function checkQuestion() {
    const fb = id('p2qFb'), guess = parseFloat(id('p2qAns').value);
    fb.style.display = 'block';
    if (!Number.isFinite(guess)) { fb.className = 'fb warn'; fb.textContent = 'Enter a number first.'; return; }
    const p = read();
    if (question.usable && !question.usable(p)) { fb.className = 'fb warn'; fb.textContent = 'This question does not apply to the current settings. Press "New question".'; return; }
    const a = question.answer(p), ok = Math.abs(guess - a) <= question.tol(a);
    answered++; if (ok) correct++;
    fb.className = 'fb ' + (ok ? 'ok' : 'no');
    fb.textContent = (ok ? 'Correct! ' : 'Not quite. ') + 'Answer: ' + fmt(a, 3) + (question.unit ? ' ' + question.unit : '') + ' (you said ' + fmt(guess, 3) + '). ' + question.explain(p, a);
    id('p2qScore').textContent = 'Session score: ' + correct + ' / ' + answered;
  }

  /* ---------- Validation (computed with the simulator, not hard-coded) ---------- */
  function validation() {
    const base = {...defaults, kind: 'first', K: 1, tau: 1, kd: 0, setpoint: 1, limit: 1000, time: 20};
    const interp = (t, y, x) => { for (let i = 1; i < t.length; i++) if (t[i] >= x) { const f = (x - t[i - 1]) / (t[i] - t[i - 1]); return y[i - 1] * (1 - f) + y[i] * f; } return y.at(-1); };
    // 1. Ideal sensor reproduces the Homework 2 check: P only, Kp = 9.
    const ideal = simulate({...base, taus: 0, kp: 9, ki: 0}, 4000);
    const y01 = interp(ideal.t, ideal.y, .1), y01exp = .9 * (1 - Math.exp(-1));
    // 2. A slow sensor does not change the steady state (P only, τs = 0.5).
    const slow = simulate({...base, taus: .5, kp: 9, ki: 0}, 4000);
    // 3. P only: ym/r = K·Kp / (τ·τs·s² + (τ+τs)s + 1 + K·Kp), a pure 2nd-order system,
    //    so the sensor overshoot must be exp(−πζ/√(1−ζ²)) with ζ = (τ+τs)/(2√(τ·τs·(1+K·Kp))).
    const zeta = 1.5 / (2 * Math.sqrt(.5 * 10)), osExp = 100 * Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta));
    const osSim = overshootOf(slow.ym, .9), osTrue = overshootOf(slow.y, .9);
    // 4. PI Kp = 4, Ki = 8: Routh gives τs,crit = 5/3 s. Simulate 20% below and 20% above it.
    const piBase = {...base, kp: 4, ki: 8, time: 40};
    const crit = criticalTaus(piBase);
    const below = loopStatus(simulate({...piBase, taus: .8 * crit}), 1).key, above = loopStatus(simulate({...piBase, taus: 1.2 * crit}), 1).key;
    const rows = [
      ['τs = 0, P only Kp = 9 (same as P1)', 'y(0.1 s) = ' + fmt(y01exp, 4) + ', y∞ = 0.9', 'y(0.1 s) = ' + fmt(y01, 4) + ', y∞ = ' + fmt(ideal.y.at(-1), 4), Math.abs(y01 - y01exp) < .005 && Math.abs(ideal.y.at(-1) - .9) < .002],
      ['τs = 0.5 s, P only Kp = 9: steady state', 'y∞ = 0.9 (sensor DC gain is 1)', 'y∞ = ' + fmt(slow.y.at(-1), 4), Math.abs(slow.y.at(-1) - .9) < .002],
      ['τs = 0.5 s, P only Kp = 9: sensor overshoot', 'ζ = ' + fmt(zeta, 3) + ' → ' + fmt(osExp, 2) + ' %', fmt(osSim, 2) + ' %', Math.abs(osSim - osExp) < .5],
      ['Same case: true y overshoots more than ym', 'true > sensor', fmt(osTrue, 1) + ' % > ' + fmt(osSim, 1) + ' %', osTrue > osSim],
      ['PI Kp = 4, Ki = 8: oscillation threshold', 'τs,crit = 5/3 = 1.667 s', 'Routh ' + fmt(crit, 3) + ' s; at 0.8×: ' + below + ', at 1.2×: ' + above, Math.abs(crit - 5 / 3) < 1e-9 && (below === 'settles' || below === 'decaying') && above === 'growing']
    ];
    id('p2validationBody').innerHTML = rows.map(([a, b, c, ok]) => '<tr><td>' + a + '</td><td>' + b + '</td><td>' + c + '</td><td><span class="pill ' + (ok ? 'pass' : 'fail') + '">' + (ok ? 'PASS' : 'FAIL') + '</span></td></tr>').join('');
    id('p2validationStatus').textContent = rows.filter(r => r[3]).length + ' / ' + rows.length + ' checks passed (actuator limit widened so the loop stays linear).';
    return rows;
  }

  /* ---------- Presets ---------- */
  const PRESETS = {
    fast: {taus: .05}, slow: {taus: .8}, osc: {taus: 1.8},
    ponly: {kp: 9, ki: 0, kd: 0, taus: .5},
  };
  function preset(name) {
    stopAnimation();
    const vals = name === 'reset' ? defaults : {...defaults, ...PRESETS[name]};
    plantKind = 'first';
    sliderIds.forEach(sid => { const key = sid.slice(2); if (key in vals) id(sid).value = vals[key]; });
    update(); newQuestion();
  }

  function init() {
    if (!id('p2')) return;
    sliderIds.forEach(name => id(name).addEventListener('input', () => { if (name === 'p2taus' && animFrame) stopAnimation(); scheduleUpdate(); }));
    // Switching plant also loads gains that suit it, so each plant starts stable
    // with an ideal sensor and reaches oscillation inside the τs slider range.
    document.querySelectorAll('#p2plantBtns button').forEach(b => b.addEventListener('click', () => {
      if (b.dataset.kind === plantKind) return;
      stopAnimation();
      const vals = b.dataset.kind === 'second' ? SECOND_DEFAULTS : defaults;
      sliderIds.forEach(sid => { const key = sid.slice(2); if (key in vals && key !== 'time') id(sid).value = vals[key]; });
      plantKind = b.dataset.kind;
      update(); newQuestion();
    }));
    id('p2presetFast').addEventListener('click', () => preset('fast'));
    id('p2presetSlow').addEventListener('click', () => preset('slow'));
    id('p2presetOsc').addEventListener('click', () => preset('osc'));
    id('p2presetP').addEventListener('click', () => preset('ponly'));
    id('p2reset').addEventListener('click', () => preset('reset'));
    id('p2animate').addEventListener('click', animate);
    id('p2qCheck').addEventListener('click', checkQuestion);
    id('p2qNew').addEventListener('click', newQuestion);
    id('p2qAns').addEventListener('keydown', ev => { if (ev.key === 'Enter') checkQuestion(); });
    update(); newQuestion(); validation();
  }
  window.P2SlowSensor = {simulate, metrics, criticalTaus, validation, update, plotSweep, read};
  init();
})();
