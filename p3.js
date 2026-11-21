/* ENCS4380 Homework 2, P3 — Digital PID (Mohammad Isam Obeidallah).
   The controller is evaluated only at t = k Ts. The actuator holds its
   previous value between samples (zero-order hold); the plant is integrated
   using forward Euler with much smaller internal time steps.
   The PID law follows the Homework 2 handout exactly, with derivative on error:
      u[k] = Kp e[k] + Ki Ts sum_{j=0..k} e[j]
                         + Kd (e[k]-e[k-1])/Ts.
   Conditional integration anti-windup is applied *before* the final clamp.
   The Arduino sketch generator below uses the same update order and limit.
*/
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const clip = (v, lo, hi) => Math.max(lo, Math.min(v, hi));
  const f = (n, p=3) => Number.isFinite(n) ? Number(n.toFixed(p)).toString() : '—';
  const ids = ['p3Ts','p3kp','p3ki','p3kd','p3K','p3tau','p3zeta','p3wn','p3setpoint','p3limit','p3time'];
  let plant='first', simulation=null, attempts=0, correct=0, questionTime=1;
  const def={Ts:.10,kp:1,ki:1,kd:0,K:1,tau:1,zeta:.5,wn:2,setpoint:1,limit:5,time:12,aw:true};
  const read=()=>({kind:plant,Ts:+$('p3Ts').value,kp:+$('p3kp').value,ki:+$('p3ki').value,
    kd:+$('p3kd').value,K:+$('p3K').value,tau:+$('p3tau').value,zeta:+$('p3zeta').value,
    wn:+$('p3wn').value,setpoint:+$('p3setpoint').value,limit:+$('p3limit').value,
    time:+$('p3time').value,aw:$('p3aw').checked});

  function simulate(p, maxPoints=1800){
    // Choose a numerical integration sub-step much smaller than the plant's
    // natural time scale; sample times remain exact integer multiples of Ts.
    const natural=p.kind==='first' ? p.tau : 1/p.wn;
    const dtMax=Math.min(0.008,natural/65,p.Ts/7);
    let y=0,v=0,I=0,prevE=0,saturated=0,peak=0;
    const t=[0], out=[0], sampleT=[],controls=[],unclipped=[],sampledY=[];
    const steps=Math.ceil(p.time/p.Ts);
    // Controller tick at the beginning of each interval, k = 0,1,...
    for(let k=0;k<steps;k++){
      const now=k*p.Ts; if(now>=p.time-1e-10)break;
      const error=p.setpoint-y;
      const de=(error-prevE)/p.Ts;
      const Icand=I+p.Ts*error;
      let raw=p.kp*error+p.ki*Icand+p.kd*de;
      // Freeze integral if saturation and error would drive it farther out.
      if(p.aw && ((raw>p.limit && error>0)||(raw< -p.limit && error<0))){
        raw=p.kp*error+p.ki*I+p.kd*de;
      }else{ I=Icand; }
      const u=clip(raw,-p.limit,p.limit);
      if(Math.abs(raw-u)>1e-9)saturated++;
      sampleT.push(now);controls.push(u);unclipped.push(raw);sampledY.push(y);
      prevE=error;
      const duration=Math.min(p.Ts,p.time-now);
      const sub=Math.max(1,Math.ceil(duration/dtMax));
      const h=duration/sub;
      for(let j=0;j<sub;j++){
        if(p.kind==='first') y+=h*(p.K*u-y)/p.tau;
        else {
          // Semi-implicit Euler for second order: y'' + 2ζωn y' + ωn² y = Kωn² u
          const acc=p.K*p.wn*p.wn*u-2*p.zeta*p.wn*v-p.wn*p.wn*y;
          v+=h*acc; y+=h*v;
        }
        if(!Number.isFinite(y) || Math.abs(y)>1e6) return {diverged:true,p};
        if(y>peak)peak=y;
        t.push(now+(j+1)*h);out.push(y);
      }
    }
    // Cap displayed values without changing the simulation itself.
    const skip=Math.max(1,Math.ceil(t.length/maxPoints));
    const tPlot=t.filter((_,i)=>i%skip===0 || i===t.length-1);
    const yPlot=out.filter((_,i)=>i%skip===0 || i===out.length-1);
    return {diverged:false,p,t:tPlot,y:yPlot,uT:sampleT,u:controls,raw:unclipped,sampledY,saturated,peak,actualFinal:y};
  }
  function crossing(t,y,level){
    for(let i=1;i<t.length;i++){
      if(y[i-1]<level && y[i]>=level){
        const d=y[i]-y[i-1];return t[i-1]+(d?(level-y[i-1])*(t[i]-t[i-1])/d:0);
      }
    }return NaN;
  }
  function performance(s){
    if(s.diverged)return {rise:NaN,overshoot:NaN,settling:NaN,ess:NaN};
    const sp=s.p.setpoint;
    const rise=crossing(s.t,s.y,.9*sp)-crossing(s.t,s.y,.1*sp);
    const overshoot=Math.max(0,(s.peak-sp)/Math.abs(sp)*100);
    const band=Math.abs(sp)*.02;
    let last=-1;for(let i=0;i<s.y.length;i++)if(Math.abs(s.y[i]-sp)>band)last=i;
    const settling=last>=s.t.length-1 ? NaN : last<0 ? 0 : s.t[last+1];
    const ess=(sp-s.actualFinal)/sp*100;
    return {rise,overshoot,settling,ess};
  }
  const PLOTLY={responsive:true,displaylogo:false,displayModeBar:false};
  function palette(){const st=getComputedStyle(document.body);return {out:st.getPropertyValue('--main').trim(),
    bright:st.getPropertyValue('--main-dark').trim(),muted:'#64748b',other:'#15803d',
    paper:st.getPropertyValue('--card').trim(),grid:st.getPropertyValue('--line').trim()};}
  function layout(x,y){const c=palette();return {paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',
    font:{family:'Inter, sans-serif',color:'#554638',size:11},margin:{l:56,r:16,b:56,t:18},
    xaxis:{title:x,gridcolor:c.grid,zeroline:false},yaxis:{title:y,gridcolor:c.grid,zeroline:false},
    legend:{orientation:'h',x:0,y:1.12},hovermode:'x unified'};}
  function showPlots(s,p){if(!window.Plotly || s.diverged)return;
    const c=palette(), ref={...p,Ts:.02}, compare=p.Ts===.02?s:simulate(ref);
    const traces=[
      {x:s.t,y:s.t.map(()=>p.setpoint),name:'Setpoint',line:{color:c.other,dash:'dash',width:2}},
      {x:s.t,y:s.y,name:'Digital PID output',line:{color:c.out,width:3}},
      {x:compare.t,y:compare.y,name:'Reference (Ts = 0.02 s)',line:{color:c.muted,dash:'dot',width:2}},
      {x:s.uT,y:s.sampledY,name:'Sampled measurements',mode:'markers',marker:{color:c.bright,size:5}}
    ];
    if(p.aw && s.saturated>0){
      const noAW=simulate({...p,aw:false});
      if(!noAW.diverged)traces.push({x:noAW.t,y:noAW.y,name:'Anti-windup OFF (comparison)',line:{color:'#9333ea',dash:'dash',width:2}});
    }
    Plotly.react('p3response',traces,layout('Time (s)','Plant output y'),PLOTLY);
    const control=[
      {x:s.uT,y:s.u,name:'Held actuator command',line:{color:c.out,width:2.6,shape:'hv'}},
      {x:[0,p.time],y:[p.limit,p.limit],name:'+Umax',line:{color:'#C62828',dash:'dash'}},
      {x:[0,p.time],y:[-p.limit,-p.limit],name:'−Umax',line:{color:'#C62828',dash:'dash'}}
    ];
    Plotly.react('p3control',control,layout('Time (s)','u[k] (held between samples)'),PLOTLY);
    // Small Ts sensitivity plot: same plant and gains, only Ts changes.
    const times=[.02,.05,.1,.2,.35,.5,.75,1];
    if(!times.some(x=>Math.abs(x-p.Ts)<1e-8))times.push(p.Ts);
    times.sort((a,b)=>a-b);
    const results=times.map(Ts=>{const q=simulate({...p,Ts},700);
      return q.diverged ? null : performance(q);});
    const errors=results.map(v=>v?Math.abs(v.ess):null);
    const overs=results.map(v=>v?v.overshoot:null);
    const selected=times.findIndex(x=>Math.abs(x-p.Ts)<1e-8);
    Plotly.react('p3sweep',[
      {x:times,y:overs,mode:'lines+markers',name:'Overshoot (%)',line:{color:c.out,width:2.6}},
      {x:times,y:errors,mode:'lines+markers',name:'|End error| (%)',line:{color:c.muted,width:2.0,dash:'dash'}},
      {x:[p.Ts],y:[overs[selected]],mode:'markers',name:'Selected Ts',marker:{color:c.other,size:12,symbol:'diamond'}}
    ],layout('Sampling time Ts (s)','Performance error (%)'),PLOTLY);
  }
  function metricsHTML(s){const p=performance(s);
    const entries=[['Rise time (10–90%)',Number.isFinite(p.rise)?f(p.rise,2)+' s':'Not reached'],
      ['Overshoot',f(p.overshoot,1)+'%'],
      ['Settling time (±2%)',Number.isFinite(p.settling)?f(p.settling,2)+' s':'Not settled'],
      ['End-of-run error',f(p.ess,2)+'%'],
      ['Saturated samples',s.saturated+' / '+s.u.length]];
    $('p3metrics').innerHTML=entries.map(([name,value])=>`<div class="stat"><div class="k">${name}</div><div class="v">${value}</div></div>`).join('');
    const slow=p.settling!==p.settling||Math.abs(p.ess)>2;
    $('p3warning').textContent=slow?'The loop has not settled to within 2% during this run. Try smaller Ts or retune the gains.':'The simulated response reaches the target within the plotted horizon.';
    $('p3warning').classList.toggle('p3-warn',slow);
  }
  function arduino(p){const ms=Math.round(p.Ts*1000);
return `// ENCS4380 HW2 P3 — generated digital PID example
// Assumptions: analog sensor on A0 outputs 0..5 V in the same
// units as the setpoint; motor uses an H-bridge, PWM pin 9.
// Calibrate the sensor and pin assignments for your hardware.
const byte SENSOR_PIN = A0;
const byte PWM_PIN = 9, IN1 = 7, IN2 = 8;
const unsigned long TS_MS = ${ms};
const float Ts = ${p.Ts.toFixed(3)}f;  // seconds
const float Kp = ${p.kp.toFixed(2)}f;
const float Ki = ${p.ki.toFixed(2)}f;
const float Kd = ${p.kd.toFixed(2)}f;
const float SETPOINT = ${p.setpoint.toFixed(2)}f;
const float UMAX = ${p.limit.toFixed(2)}f;
const bool ANTI_WINDUP = ${p.aw?'true':'false'};
float integral = 0.0f, prevError = 0.0f;
unsigned long lastTick = 0;

float clampControl(float x) {
  return constrain(x, -UMAX, UMAX);
}
void driveMotor(float u) {
  digitalWrite(IN1, u >= 0 ? HIGH : LOW);
  digitalWrite(IN2, u >= 0 ? LOW : HIGH);
  int pwm = (int)(255.0f * fabs(u) / UMAX + 0.5f);
  analogWrite(PWM_PIN, constrain(pwm, 0, 255));
}
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(9600);
  lastTick = millis();
}
void loop() {
  unsigned long now = millis();
  if (now - lastTick < TS_MS) return;
  lastTick += TS_MS;
  // Change scaling if the feedback sensor is not 0..5 units.
  float measured = analogRead(SENSOR_PIN) * (5.0f / 1023.0f);
  float error = SETPOINT - measured;
  float de = (error - prevError) / Ts;
  float candidate = integral + error * Ts;
  float raw = Kp*error + Ki*candidate + Kd*de;
  // Conditional integration prevents deeper saturation.
  if (ANTI_WINDUP &&
      ((raw > UMAX && error > 0) ||
       (raw < -UMAX && error < 0))) {
    raw = Kp*error + Ki*integral + Kd*de;
  } else {
    integral = candidate;
  }
  float u = clampControl(raw); // zero-order held until next tick
  driveMotor(u);
  prevError = error;
  Serial.print(measured); Serial.print(',');
  Serial.println(u);
}`;}
  function questionReset(){ $('p3qFeedback').className='fb'; $('p3qFeedback').textContent=''; $('p3qAnswer').value='';
    $('p3qTime').textContent=f(questionTime,1);$('p3qScore').textContent=`Session score: ${correct} / ${attempts}`; }
  function update(){const p=read();
    ids.forEach(id=>{const v=$(id); const el=$(id+'V');if(el)el.textContent=v.value;});
    $('p3first').style.display=plant==='first'?'':'none';
    $('p3second').style.display=plant==='second'?'':'none';
    $('p3plantEq').textContent=plant==='first'?'τ dy/dt + y = K u':'y″ + 2ζωₙ y′ + ωₙ²y = Kωₙ²u';
    simulation=simulate(p);if(simulation.diverged){$('p3warning').textContent='Simulation exceeded safe bounds; reduce the gains.';return;}
    showPlots(simulation,p);metricsHTML(simulation);
    $('p3arduino').textContent=arduino(p);
    $('p3sampling').textContent=`A new control command is calculated every ${f(p.Ts,2)} s; between samples it remains constant. ${p.aw?'Anti-windup is ON.':'Anti-windup is OFF; integral action can accumulate during saturation.'}`;
    questionReset();
  }
  function reset(){plant='first';for(const [key,val] of Object.entries(def)){
    if(key==='aw')$('p3aw').checked=val;
    else $(key==='Ts'?'p3Ts':'p3'+key).value=val;
  } document.querySelectorAll('#p3plantBtns button').forEach(b=>b.classList.toggle('on',b.dataset.kind==='first'));
    update();}
  document.querySelectorAll('#p3plantBtns button').forEach(b=>b.addEventListener('click',()=>{
    plant=b.dataset.kind;document.querySelectorAll('#p3plantBtns button').forEach(x=>x.classList.toggle('on',x===b));update();
  }));
  ids.forEach(id=>$(id).addEventListener('input',update));
  $('p3aw').addEventListener('change',update);
  $('p3reset').addEventListener('click',reset);
  $('p3presetFast').addEventListener('click',()=>{$('p3Ts').value=.02;update();});
  $('p3presetSlow').addEventListener('click',()=>{$('p3Ts').value=.75;update();});
  $('p3presetWindup').addEventListener('click',()=>{
    plant='first';document.querySelectorAll('#p3plantBtns button').forEach(b=>b.classList.toggle('on',b.dataset.kind==='first'));
    $('p3Ts').value=.1;$('p3kp').value=2;$('p3ki').value=5;$('p3kd').value=0;
    $('p3limit').value=2;$('p3setpoint').value=1.5;$('p3time').value=20;update();
  });
  $('p3new').addEventListener('click',()=>{questionTime=1+Math.floor(Math.random()*4);questionReset();});
  $('p3check').addEventListener('click',()=>{
    const guess=Number($('p3qAnswer').value);if(!$('p3qAnswer').value.trim()||!Number.isFinite(guess))return;
    if(!simulation || simulation.diverged)return;
    const a=simulation.t,b=simulation.y;
    let k=1;while(k<a.length-1&&a[k]<questionTime)k++;
    const val=a[k-1]===a[k]?b[k] : b[k-1]+(b[k]-b[k-1])*(questionTime-a[k-1])/(a[k]-a[k-1]);
    const pass=Math.abs(guess-val)<=Math.max(.05,.07*Math.abs(val));
    attempts++;if(pass)correct++;
    $('p3qFeedback').className='fb '+(pass?'ok':'no');
    $('p3qFeedback').textContent=`${pass?'Correct!':'Not quite.'} Actual y(${f(questionTime,1)} s) = ${f(val,3)}. Your prediction: ${f(guess,3)}.`;
    $('p3qScore').textContent=`Session score: ${correct} / ${attempts}`;
  });
  $('p3qAnswer').addEventListener('keydown',e=>{if(e.key==='Enter')$('p3check').click();});
  $('p3copy').addEventListener('click',async()=>{
    const code=$('p3arduino').textContent;
    try{await navigator.clipboard.writeText(code);$('p3copy').textContent='Copied!';}
    catch(e){const r=document.createRange();r.selectNodeContents($('p3arduino'));const s=window.getSelection();s.removeAllRanges();s.addRange(r);$('p3copy').textContent='Select and press Ctrl+C';}
  });
  window.P3DigitalPID={update,simulate,performance};
  update();
})();
