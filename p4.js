/* =========================================================
   P4: PID tuning challenge (wrapped so its names cannot clash with P1-P3)
   ========================================================= */
(function(){
function boot(){
'use strict';

const $ = id => document.getElementById(id);
const css = v => getComputedStyle(document.body).getPropertyValue(v).trim();
const cfg = {responsive:true, displayModeBar:false};
function fmt(x, d = 3){ return isFinite(x) ? parseFloat(Number(x).toFixed(d)).toString() : '—'; }
const rnd = (a, b, d = 1) => +(a + Math.random() * (b - a)).toFixed(d);
function randn(){ let u = 0, v = 0; while(!u) u = Math.random(); while(!v) v = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
const linspace = (a, b, n) => Array.from({length:n}, (_, i) => a + (b - a) * i / (n - 1));
function nice(x){ return x >= 10 ? Math.ceil(x) : x >= 1 ? Math.ceil(x * 10) / 10 : Math.ceil(x * 100) / 100; }

const G = {pl:null, ref:null, spec:null, ol:null, answered:false, hint:0, rounds:0, points:0};

/* ---------------- closed-loop simulation (RK4) ----------------
   States: first order  x = [y, I, yf]       second order  x = [y, v, I, yf]
   I  = Ki * integral of e,  yf = low-pass filtered y (for the derivative). */
function simulate(pl, g, T){
  const n = 5000, h = T / n, tf = 3 * h, so = pl.type === 'so';
  const iI = so ? 2 : 1, iF = so ? 3 : 2;
  const ctrl = x => g.Kp * (1 - x[0]) + x[iI] - g.Kd * (x[0] - x[iF]) / tf;
  const f = x => {
    const u = ctrl(x), dx = new Array(x.length);
    if(so){ dx[0] = x[1]; dx[1] = pl.wn * pl.wn * (pl.K * u - x[0]) - 2 * pl.zeta * pl.wn * x[1]; }
    else dx[0] = (pl.K * u - x[0]) / pl.tau;
    dx[iI] = g.Ki * (1 - x[0]);
    dx[iF] = (x[0] - x[iF]) / tf;
    return dx;
  };
  let x = new Array(so ? 4 : 3).fill(0), blown = false;
  const t = [], y = [], u = [];
  for(let k = 0; k <= n; k++){
    t.push(k * h); y.push(x[0]); u.push(ctrl(x));
    if(k === n) break;
    const k1 = f(x), k2 = f(x.map((v, i) => v + h * k1[i] / 2)), k3 = f(x.map((v, i) => v + h * k2[i] / 2)), k4 = f(x.map((v, i) => v + h * k3[i]));
    x = x.map((v, i) => v + h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6);
    if(!isFinite(x[0]) || Math.abs(x[0]) > 1e4){ blown = true; break; }
  }
  return {t, y, u, blown, T};
}

function crossTime(t, y, level){
  for(let i = 1; i < y.length; i++) if(y[i-1] < level && y[i] >= level) return t[i-1] + (level - y[i-1]) / (y[i] - y[i-1]) * (t[i] - t[i-1]);
  return null;
}
function analyse(s){
  if(s.blown) return {blown:true, os:Infinity, settle:null, sse:Infinity, peakU:Infinity, rise:null};
  const t = s.t, y = s.y;
  let peak = -Infinity; for(const v of y) if(v > peak) peak = v;
  let last = -1; for(let i = 0; i < y.length; i++) if(Math.abs(y[i] - 1) > 0.02) last = i;
  let peakU = 0; for(const v of s.u) if(Math.abs(v) > peakU) peakU = Math.abs(v);
  const t10 = crossTime(t, y, 0.1), t90 = crossTime(t, y, 0.9);
  return {blown:false, os:Math.max(0, (peak - 1) * 100),
          settle: last === y.length - 1 ? null : (last < 0 ? 0 : t[last + 1]),
          sse: 1 - y[y.length - 1], peakU, rise: (t10 !== null && t90 !== null) ? t90 - t10 : null};
}

/* ---------------- a reference design that always meets the spec ----------------
   First order: PI with closed-loop wn = 2/tau and zeta = 1.2.
   Second order: PID by pole placement at (s + p)(s^2 + 2*zc*wc*s + wc^2),
   with wc = 1.5*wn, zc = 0.8, p = wc. */
function refGains(pl){
  if(pl.type === 'fo') return {Kp:3.8 / pl.K, Ki:4 / (pl.K * pl.tau), Kd:0};
  const wc = 1.5 * pl.wn, zc = 0.8, p = wc;
  const a2 = p + 2 * zc * wc, a1 = wc * wc + 2 * zc * wc * p, a0 = p * wc * wc, w2 = pl.wn * pl.wn;
  return {Kp:a1 / w2 - 1, Ki:a0 / w2, Kd:(a2 - 2 * pl.zeta * pl.wn) / w2};
}

/* ---------------- a new round ---------------- */
function openLoop(pl){
  const t = linspace(0, pl.T, 160);
  const clean = t.map(s => {
    if(pl.type === 'fo') return pl.K * (1 - Math.exp(-s / pl.tau));
    const z = pl.zeta, wd = pl.wn * Math.sqrt(1 - z * z);
    return 1 - Math.exp(-z * pl.wn * s) / Math.sqrt(1 - z * z) * Math.sin(wd * s + Math.acos(z));
  });
  const sigma = 0.012 * pl.K;
  return {t, y: clean.map(v => v + sigma * randn())};
}

function newRound(){
  let pl, ref, rm, tries = 0;
  do {
    pl = Math.random() < 0.5 ? {type:'fo', K:rnd(0.5, 5, 1), tau:rnd(1, 10, 1)}
                             : {type:'so', K:1, zeta:rnd(0.1, 0.75, 2), wn:rnd(1, 10, 1)};
    const Tc = pl.type === 'fo' ? pl.tau : 1 / (pl.zeta * pl.wn);
    pl.T = Math.max(10 * Tc, 5);
    ref = refGains(pl);
    rm = analyse(simulate(pl, ref, pl.T));
    tries++;
  } while(tries < 30 && (rm.blown || rm.settle === null || Math.abs(rm.sse) > 0.01));

  G.pl = pl; G.ref = ref; G.answered = false; G.hint = 0;
  G.spec = {os:Math.max(10, Math.ceil(rm.os + 5)), settle:nice(rm.settle * 1.5), umax:nice(rm.peakU * 1.5)};
  G.ol = openLoop(pl);

  $('p4Kp').value = 1; $('p4Ki').value = 0; $('p4Kd').value = 0;
  $('p4hints').innerHTML = ''; $('p4fb').className = 'fb'; $('p4fb').innerHTML = '';
  $('p4revealCard').style.display = 'none';
  $('p4submit').disabled = false; $('p4hintBtn').disabled = false;
  drawOL();
  update();
  newQ();
}

/* ---------------- drawing ---------------- */
function lay(xt, yt, extra){
  return Object.assign({
    margin:{l:56, r:16, t:12, b:50},
    xaxis:{title:{text:xt}, gridcolor:'#F6E7D6', zeroline:false},
    yaxis:{title:{text:yt}, gridcolor:'#F6E7D6', zeroline:false},
    legend:{orientation:'h', x:0, y:-0.25, yanchor:'top'},
    paper_bgcolor:'rgba(0,0,0,0)', plot_bgcolor:'#fff', showlegend:false,
    font:{family:'Inter, system-ui, sans-serif', size:12, color:'#2B1608'}, hovermode:'closest'
  }, extra || {});
}
function drawOL(){
  if(!window.Plotly || !G.ol) return;
  Plotly.react('p4plotOL', [{x:G.ol.t, y:G.ol.y, mode:'markers', marker:{size:4, color:css('--main')},
    hovertemplate:'t = %{x:.3f} s<br>y = %{y:.3f}<extra></extra>'}], lay('time t (s)', 'plant output y'), cfg);
}
function gains(){ return {Kp:+$('p4Kp').value, Ki:+$('p4Ki').value, Kd:+$('p4Kd').value}; }

function checkSpec(m){
  const s = G.spec;
  return {
    os: !m.blown && m.os <= s.os,
    settle: !m.blown && m.settle !== null && m.settle <= s.settle,
    sse: !m.blown && Math.abs(m.sse) <= 0.01,
    u: !m.blown && m.peakU <= s.umax
  };
}
const pill = ok => `<span class="pill ${ok ? 'pass' : 'fail'}">${ok ? 'PASS' : 'FAIL'}</span>`;

function update(){
  const g = gains();
  $('p4Kpv').textContent = fmt(g.Kp, 2); $('p4Kiv').textContent = fmt(g.Ki, 2); $('p4Kdv').textContent = fmt(g.Kd, 2);
  if(!G.pl) return;
  const sim = simulate(G.pl, g, G.pl.T), m = analyse(sim), ok = checkSpec(m), s = G.spec;
  const rows = [
    ['Overshoot', '≤ ' + s.os + ' %', m.blown ? 'Unstable' : fmt(m.os, 1) + ' %', ok.os],
    ['Settling time (2%)', '≤ ' + s.settle + ' s', m.blown ? 'Unstable' : m.settle === null ? 'Does not settle' : fmt(m.settle, 2) + ' s', ok.settle],
    ['Steady-state error', '≤ 1 %', m.blown ? 'Unstable' : fmt(Math.abs(m.sse) * 100, 2) + ' %', ok.sse],
    ['Peak control signal |u|', '≤ ' + s.umax, m.blown ? 'Unstable' : fmt(m.peakU, 2), ok.u]
  ];
  $('p4specBody').innerHTML = rows.map(([a, b, c, p]) => `<tr><td>${a}</td><td>${b}</td><td>${c}</td><td>${pill(p)}</td></tr>`).join('');
  const n = Object.values(ok).filter(Boolean).length;
  $('p4specSummary').textContent = `Passing ${n} of 4 parts` + (n === 4 ? '. Ready to submit!' : '.');
  if(!window.Plotly) return;

  const view = Math.min(G.pl.T, 3 * s.settle);
  const shapes = [
    {type:'rect', xref:'x', yref:'y', x0:0, x1:G.pl.T, y0:0.98, y1:1.02, fillcolor:'rgba(255,153,19,.25)', line:{width:0}},
    {type:'line', xref:'x', yref:'paper', x0:s.settle, x1:s.settle, y0:0, y1:1, line:{color:css('--main-dark'), dash:'dot', width:2}},
    {type:'line', xref:'paper', yref:'y', x0:0, x1:1, y0:1 + s.os / 100, y1:1 + s.os / 100, line:{color:'#C62828', dash:'dash', width:1.5}}
  ];
  const L = lay('time t (s)', 'output y', {shapes});
  L.xaxis.range = [0, view];
  Plotly.react('p4plotY', [
    {x:sim.t, y:sim.y, line:{color:css('--main'), width:3.5}, hovertemplate:'t = %{x:.3f} s<br>y = %{y:.3f}<extra></extra>'},
    {x:[0, G.pl.T], y:[1, 1], line:{color:'#2B1608', dash:'dot', width:1.5}, hoverinfo:'skip'}
  ], L, cfg);

  const L2 = lay('time t (s)', 'control u', {shapes:[
    {type:'line', xref:'paper', yref:'y', x0:0, x1:1, y0:s.umax, y1:s.umax, line:{color:css('--main-dark'), dash:'dash', width:1.5}}
  ]});
  L2.xaxis.range = [0, view];
  if(!m.blown) L2.yaxis.range = [Math.min(0, ...sim.u) - 0.05 * s.umax, Math.max(1.3 * s.umax, 1.05 * m.peakU)];
  Plotly.react('p4plotU', [{x:sim.t, y:sim.u, line:{color:css('--main'), width:2.5}, hovertemplate:'t = %{x:.3f} s<br>u = %{y:.3f}<extra></extra>'}], L2, cfg);
}

/* ---------------- submit, reveal, hints ---------------- */
function submit(){
  if(G.answered) return;
  const g = gains(), m = analyse(simulate(G.pl, g, G.pl.T)), ok = checkSpec(m);
  const names = {os:'overshoot', settle:'settling time', sse:'steady-state error', u:'control-signal peak'};
  const passed = Object.keys(ok).filter(k => ok[k]), failed = Object.keys(ok).filter(k => !ok[k]);
  const pts = passed.length * 25;
  G.answered = true; G.rounds++; G.points += pts;
  $('p4submit').disabled = true; $('p4hintBtn').disabled = true;

  $('p4round').textContent = G.rounds; $('p4last').textContent = pts;
  $('p4total').textContent = G.points; $('p4max').textContent = G.rounds * 100;

  const pl = G.pl, r = G.ref;
  const plantTxt = pl.type === 'fo' ? `First order: K = ${fmt(pl.K, 2)}, τ = ${fmt(pl.tau, 2)} s`
                                    : `Second order: K = 1, ζ = ${fmt(pl.zeta, 2)}, ωₙ = ${fmt(pl.wn, 2)} rad/s`;
  $('p4revealStats').innerHTML = [
    ['Hidden plant', plantTxt],
    ['A design that meets the spec', `Kp = ${fmt(r.Kp, 2)}, Ki = ${fmt(r.Ki, 2)}, Kd = ${fmt(r.Kd, 2)}`],
    ['Your gains', `Kp = ${fmt(g.Kp, 2)}, Ki = ${fmt(g.Ki, 2)}, Kd = ${fmt(g.Kd, 2)}`]
  ].map(([k, v]) => `<div class="stat"><div class="k">${k}</div><div class="v">${v}</div></div>`).join('');
  $('p4revealSummary').textContent = `This round: ${pts} / 100` + (pts === 100 ? ' (full marks)' : '');
  $('p4revealCard').style.display = '';

  const fb = $('p4fb');
  fb.className = 'fb ' + (pts === 100 ? 'ok' : 'no');
  fb.innerHTML = pts === 100 ? '<b>Excellent.</b> Every part of the spec passes.'
    : `<b>${passed.length} of 4 parts passed.</b> Failed: ${failed.map(k => names[k]).join(', ')}.`;
}

const HINTS = [
  'Start with Kp only (Ki = 0, Kd = 0). Raise it until the response is fast enough, but watch the control-signal limit.',
  'Proportional control alone always leaves a steady-state error. Add Ki to remove it. Too much Ki makes the overshoot grow.',
  'Overshoot too high? Lower Ki, or add a little Kd. It acts on the measured output and calms the oscillation.',
  'A bigger plant gain K means you need a smaller Kp for the same loop gain K·Kp. Estimate K from the open-loop test first.'
];
function showHint(){
  if(G.hint >= HINTS.length) return;
  const li = document.createElement('li'); li.textContent = HINTS[G.hint++];
  $('p4hints').appendChild(li);
  if(G.hint >= HINTS.length) $('p4hintBtn').disabled = true;
}

/* ---------------- predict, then check ---------------- */
const Q = {type:'ki', done:false, right:0, total:0};
function renderQ(){
  const g = gains();
  let opts;
  if(Q.type === 'ki'){
    const to = g.Ki > 0 ? Math.min(2 * g.Ki, 40) : 1;
    $('p4qText').innerHTML = `Same Kp and Kd. Predict: if I change Ki from ${fmt(g.Ki, 2)} to ${fmt(to, 2)}, the <b>overshoot</b> will…`;
    opts = [['up', 'increase'], ['down', 'decrease'], ['same', 'stay about the same']];
  } else {
    $('p4qText').innerHTML = `Set Ki = 0 and Kd = 0 (proportional control only) and keep Kp = ${fmt(g.Kp, 2)}. Predict: will the output end within 1% of the setpoint?`;
    opts = [['yes', 'yes'], ['no', 'no']];
  }
  $('p4qOpts').innerHTML = opts.map(([v, l]) => `<label class="opt"><input type="radio" name="p4q" value="${v}"> ${l}</label>`).join('');
}
function newQ(){
  Q.type = Math.random() < 0.5 ? 'ki' : 'p'; Q.done = false;
  $('p4qFb').className = 'fb'; $('p4qFb').innerHTML = '';
  renderQ();
}
function checkQ(){
  const sel = document.querySelector('input[name=p4q]:checked'), fb = $('p4qFb');
  if(!sel){ fb.className = 'fb warn'; fb.textContent = 'Pick an answer first.'; return; }
  const g = gains(), T = G.pl.T;
  let truth, why;
  if(Q.type === 'ki'){
    const to = g.Ki > 0 ? Math.min(2 * g.Ki, 40) : 1;
    const a = analyse(simulate(G.pl, g, T)), b = analyse(simulate(G.pl, Object.assign({}, g, {Ki:to}), T));
    const diff = (!isFinite(a.os) && !isFinite(b.os)) ? 0 : b.os - a.os;
    truth = diff > 1 ? 'up' : diff < -1 ? 'down' : 'same';
    why = `Overshoot went from ${fmt(a.os, 1)}% to ${fmt(b.os, 1)}%. The integral term removes steady-state error, but it also adds lag in the loop, so more Ki usually means more overshoot.`;
  } else {
    const m = analyse(simulate(G.pl, {Kp:g.Kp, Ki:0, Kd:0}, T));
    truth = (!m.blown && Math.abs(m.sse) <= 0.01) ? 'yes' : 'no';
    why = `With P only, the steady-state error is ${fmt(Math.abs(m.sse) * 100, 2)}% of the setpoint. For a plant with gain K it equals 1/(1 + K·Kp) for a first-order plant, so a large Kp makes it small but never exactly zero. Only the integral term removes it.`;
  }
  const ok = sel.value === truth;
  if(!Q.done){ Q.total++; if(ok) Q.right++; }
  Q.done = true;
  const label = {up:'increase', down:'decrease', same:'stay about the same', yes:'yes', no:'no'}[truth];
  fb.className = 'fb ' + (ok ? 'ok' : 'no');
  fb.innerHTML = `<b>${ok ? 'Good prediction!' : 'Not quite.'}</b> The answer is: ${label}.<br>${why}`;
  $('p4qScore').textContent = `Score this session: ${Q.right} / ${Q.total}`;
}

/* ---------------- wiring ---------------- */
['p4Kp', 'p4Ki', 'p4Kd'].forEach(id => $(id).addEventListener('input', () => { update(); if(!Q.done) renderQ(); }));
$('p4submit').addEventListener('click', submit);
$('p4hintBtn').addEventListener('click', showHint);
$('p4newBtn').addEventListener('click', newRound);
$('p4zeroBtn').addEventListener('click', () => { $('p4Kp').value = 1; $('p4Ki').value = 0; $('p4Kd').value = 0; update(); if(!Q.done) renderQ(); });
$('p4qCheck').addEventListener('click', checkQ);
$('p4qNew').addEventListener('click', newQ);

/* ---------------- start-up ---------------- */
function ready(){ drawOL(); update(); }
window.p4Redraw = ready;   // call this when Plotly has loaded and when the P4 page is shown
newRound();

}
/* run once the page's HTML exists, so the script tag can go anywhere */
if(document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot); else boot();
})();