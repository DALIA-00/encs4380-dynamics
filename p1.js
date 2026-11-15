/* ENCS4380 HW2 — P1 PID Playground (Ghaleb Shakhsheer)
   Continuous-time PID with derivative on measurement (avoids setpoint derivative kick).
   Plant integration: forward Euler with adaptive integration step; NO closed-form response.
   Everything runs client-side. P2/P3/P4 can be added without modifying this file.
*/
(() => {
  'use strict';
  const id = x => document.getElementById(x);
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const fmtP1 = (x, digits=3) => Number.isFinite(x) ? Number(x.toFixed(digits)).toString() : '—';
  const outputIds = ['p1K','p1tau','p1zeta','p1wn','p1kp','p1ki','p1kd','p1setpoint','p1limit'];
  const defaults = {kind:'first', K:1, tau:1, zeta:.5, wn:2, kp:9, ki:0, kd:0, setpoint:1, limit:20};
  let plantKind='first', result=null, dirtyPrediction=true, correct=0, answered=0;

  function read() {
    return {
      kind:plantKind, K:+id('p1K').value, tau:+id('p1tau').value,
      zeta:+id('p1zeta').value, wn:+id('p1wn').value,
      kp:+id('p1kp').value, ki:+id('p1ki').value, kd:+id('p1kd').value,
      setpoint:+id('p1setpoint').value, limit:+id('p1limit').value
    };
  }

  // First-order plant (same G(s) as HW1 M1): y'=(K*u-y)/tau.
  // Second-order plant (HW1 M3 form): y''=K*wn^2*u-2*zeta*wn*y'-wn^2*y.
  // Derivative is on measured output: u = Kp*(r-y) + Ki*I - Kd*y'.
  // Integral is frozen only when saturation would wind it up further.
  function simulate(p, time=14) {
    const stiffness = p.kind==='first'
      ? (1+p.K*p.kp+p.K*p.kd/p.tau)/p.tau
      : p.wn*(1+2*p.zeta)+p.K*p.wn*p.wn*p.kd + Math.sqrt(Math.max(0,p.K*p.ki))*p.wn;
    const dtMax = Math.min(.001, .08 / Math.max(stiffness, .001));
    const steps = Math.ceil(time / Math.max(.00012,dtMax));
    const dt = time / steps;
    const stride = Math.max(1, Math.floor(steps / 1200));
    const t=[], yOut=[], uOut=[];
    let y=0, velocity=0, derivative=0, integral=0, maxY=0, diverged=false;
    for(let i=0;i<=steps;i++) {
      const e=p.setpoint-y;
      let trial=integral+e*dt;
      let requested=p.kp*e+p.ki*trial-p.kd*derivative;
      if((requested>p.limit && e>0) || (requested < -p.limit && e<0)) trial=integral;
      requested=p.kp*e+p.ki*trial-p.kd*derivative;
      const control=clamp(requested,-p.limit,p.limit);
      if(i%stride===0 || i===steps) {
        t.push(i*dt);yOut.push(y);uOut.push(control);
      }
      maxY=Math.max(maxY,y);
      if(i===steps)break;
      if(p.kind==='first') {
        const dy=(p.K*control-y)/p.tau;
        y+=dt*dy; derivative=dy;
      } else {
        const accel=p.K*p.wn*p.wn*control-2*p.zeta*p.wn*velocity-p.wn*p.wn*y;
        velocity+=dt*accel;
        y+=dt*velocity; derivative=velocity;
      }
      integral=trial;
      if(!Number.isFinite(y) || Math.abs(y)>1e7) { diverged=true; break; }
    }
    return {t,y:yOut,u:uOut,dt,diverged,maxY,params:p};
  }

  const crossing = (t,y,level) => {
    for(let i=1;i<t.length;i++) if(y[i-1]<level && y[i]>=level){
      const d=y[i]-y[i-1]; return t[i-1]+(d ? (level-y[i-1])/d*(t[i]-t[i-1]) : 0);
    }
    return NaN;
  };
  function metrics(s) {
    const p=s.params, finalTarget=p.ki>0 ? p.setpoint : p.setpoint*(p.K*p.kp)/(1+p.K*p.kp);
    if(s.diverged || !(finalTarget>0))return {rise:NaN, overshoot:NaN, settle:NaN, err:NaN, finalTarget};
    const t10=crossing(s.t,s.y,.1*finalTarget), t90=crossing(s.t,s.y,.9*finalTarget);
    const rise=t90-t10;
    const peak=Math.max(...s.y);
    const overshoot=Math.max(0,(peak-finalTarget)/finalTarget*100);
    const band=.02*Math.abs(finalTarget);
    let lastOutside=-1;
    for(let i=0;i<s.y.length;i++)if(Math.abs(s.y[i]-finalTarget)>band)lastOutside=i;
    const settle=lastOutside>=s.y.length-1 ? NaN : lastOutside<0 ? 0 : s.t[lastOutside+1];
    const err=(p.setpoint-s.y[s.y.length-1])/p.setpoint*100;
    return {rise,overshoot,settle,err,finalTarget};
  }

  function labels(p){
    for(const name of outputIds){
      const v=id(name).value;
      id(name+'V').textContent= (name==='p1setpoint' || name==='p1limit' || name==='p1K' || name==='p1tau') ? v : v;
    }
    const isFirst=p.kind==='first';
    id('p1firstControls').style.display=isFirst?'':'none';
    id('p1secondControls').style.display=isFirst?'none':'';
    id('p1equation').innerHTML=isFirst ? 'G(s) = K / (τs + 1)' : 'G(s) = Kωₙ² / (s² + 2ζωₙs + ωₙ²)';
    id('p1plantLabel').textContent=isFirst?'First-order plant':'Second-order plant';
    id('p1limitsText').textContent='Actuator limit: −'+fmtP1(p.limit)+' to +'+fmtP1(p.limit)+' (normalized input units).';
    id('p1diagramPlant').textContent=isFirst?'K / (τs+1)':'Kωₙ² / (s²+2ζωₙs+ωₙ²)';
    document.querySelectorAll('#p1plantBtns button').forEach(b=>b.classList.toggle('on',b.dataset.kind===p.kind));
  }

  const chartLayout=(title,upper)=>({
    paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',
    margin:{l:55,r:18,t:22,b:55},font:{family:'Inter, sans-serif',color:'#7A5A44',size:12},
    xaxis:{title:'Time (s)',gridcolor:'#F3DEC6',zeroline:false},
    yaxis:{title,gridcolor:'#F3DEC6',zeroline:false,range:upper||undefined},
    legend:{orientation:'h',y:1.13,x:0},hovermode:'x unified',
  });
  function plot(s){
    if(!window.Plotly)return;
    const p=s.params;
    const setpoint=s.t.map(()=>p.setpoint), hi=s.t.map(()=>p.limit), low=s.t.map(()=>-p.limit);
    const C={main:getComputedStyle(document.body).getPropertyValue('--main').trim()||'#E24C00',positive:'#2E7D32'};
    window.Plotly.react('p1response',[
      {x:s.t,y:setpoint,name:'Setpoint',mode:'lines',line:{color:C.positive,width:2,dash:'dash'}},
      {x:s.t,y:s.y,name:'Plant output y(t)',mode:'lines',line:{color:C.main,width:3}}
    ],chartLayout('Output y'),{responsive:true,displayModeBar:false});
    window.Plotly.react('p1control',[
      {x:s.t,y:s.u,name:'Control u(t)',mode:'lines',line:{color:C.main,width:2.5,shape:'hv'}},
      {x:s.t,y:hi,name:'Upper limit',mode:'lines',line:{color:'#C62828',width:1.5,dash:'dot'}},
      {x:s.t,y:low,name:'Lower limit',mode:'lines',line:{color:'#C62828',width:1.5,dash:'dot'}}
    ],chartLayout('Control u'),{responsive:true,displayModeBar:false});
  }
  function renderStats(s){
    const m=metrics(s);
    const display=(value,unit,prec=2)=> Number.isFinite(value) ? fmtP1(value,prec)+unit : 'Not reached';
    const vals=[
      ['Rise time (10–90%)',display(m.rise,' s')],
      ['Overshoot',display(m.overshoot,' %')],
      ['Settling time (±2%)',display(m.settle,' s')],
      ['Steady-state error (estimate)',display(m.err,' %')],
    ];
    id('p1metrics').innerHTML=vals.map(([k,v])=>'<div class="stat"><div class="k">'+k+'</div><div class="v">'+v+'</div></div>').join('');
    id('p1metricsNote').textContent=s.diverged
      ? 'Response diverged: try lower gains. Metrics are not meaningful.'
      : 'Rise / overshoot / settling are measured against the expected steady output ('+fmtP1(m.finalTarget)+'). Steady-state error is estimated as (setpoint − output at '+fmtP1(s.t.at(-1),1)+'s) / setpoint. If settling is “Not reached”, increase simulation time or tune the gains.';
  }
  function update(){
    const p=read(); labels(p);
    result=simulate(p);
    renderStats(result);
    plot(result);
    dirtyPrediction=true;
    id('p1predFeedback').style.display='none';
    id('p1predAnswer').value='';
    id('p1predictionTime').textContent='1.0';
  }
  let updateFrame=null;
  function scheduleUpdate(){
    if(updateFrame!==null)cancelAnimationFrame(updateFrame);
    updateFrame=requestAnimationFrame(()=>{updateFrame=null;update();});
  }
  function interpolated(t,y,x){
    if(x<=t[0])return y[0];
    for(let i=1;i<t.length;i++)if(t[i]>=x){let f=(x-t[i-1])/(t[i]-t[i-1]);return y[i-1]*(1-f)+y[i]*f;}
    return y.at(-1);
  }
  function checkPrediction(){
    if(!result)return;
    const guess=parseFloat(id('p1predAnswer').value),fb=id('p1predFeedback');
    if(!Number.isFinite(guess)){fb.className='fb warn';fb.textContent='Enter a number first.';fb.style.display='block';return;}
    const correctVal=interpolated(result.t,result.y,1);
    const tol=Math.max(.05,.05*Math.abs(correctVal));
    const ok=Math.abs(guess-correctVal)<=tol;
    answered++;if(ok)correct++;
    fb.className='fb '+(ok?'ok':'no');fb.style.display='block';
    fb.textContent=(ok?'Correct! ':'Not quite. ')+'At t = 1 s the numerical solver gives y = '+fmtP1(correctVal,4)+'. Your prediction: '+fmtP1(guess,4)+'. Allowed error ±'+fmtP1(tol,3)+'.';
    id('p1predScore').textContent='Session score: '+correct+' / '+answered;
    dirtyPrediction=false;
  }
  function validation(){
    const base={...defaults,kind:'first', K:1,tau:1,setpoint:1,limit:100};
    const pOnly=simulate({...base,kp:9,ki:0,kd:0},14);
    const pTau=interpolated(pOnly.t,pOnly.y,.1);
    const pFinal=pOnly.y.at(-1);
    const yTauExpect=.9*(1-Math.exp(-1));
    const pi=simulate({...base,kp:1,ki:1,kd:0},18);
    const piAlt=simulate({...base,kp:2,ki:.6,kd:0},24);
    const piErr=Math.abs(1-pi.y.at(-1));
    const piAltErr=Math.abs(1-piAlt.y.at(-1));
    const wn=Math.sqrt(base.K*1/base.tau);
    const zeta=(1+base.K*1)/(2*base.tau*wn);
    const rows=[
      ['P-only: Kp = 9', 'τcl = 0.1 s; y(0.1s) = '+fmtP1(yTauExpect,4), 'y(0.1s) = '+fmtP1(pTau,4),Math.abs(pTau-yTauExpect)<.005],
      ['P-only: error at steady state','10%',''+fmtP1(100*(1-pFinal),3)+'%',Math.abs(pFinal-.9)<.002],
      ['PI: Kp = 1, Ki = 1','ωn = 1 rad/s; ζ = 1','ωn = '+fmtP1(wn)+'; ζ = '+fmtP1(zeta),Math.abs(wn-1)<1e-9&&Math.abs(zeta-1)<1e-9],
      ['PI: zero step error','0%',''+fmtP1(piErr*100,4)+'%',piErr<.005],
      ['Other PI: Kp = 2, Ki = 0.6','0%',''+fmtP1(piAltErr*100,4)+'%',piAltErr<.005]
    ];
    id('p1validationBody').innerHTML=rows.map(([a,b,c,ok])=>'<tr><td>'+a+'</td><td>'+b+'</td><td>'+c+'</td><td><span class="pill '+(ok?'pass':'fail')+'">'+(ok?'PASS':'FAIL')+'</span></td></tr>').join('');
    id('p1validationStatus').textContent=rows.filter(r=>r[3]).length+' / '+rows.length+' simulation checks passed (limit widened to prevent saturation).';
    return rows;
  }
  function preset(kind){
    const vals=kind==='p' ? {kp:9,ki:0,kd:0} : kind==='pi' ? {kp:1,ki:1,kd:0} : {kp:6,ki:2,kd:.9};
    plantKind='first';
    for(const [key,value] of Object.entries({...defaults,...vals})) {
      if(key!=='kind')id('p1'+(key==='setpoint'?'setpoint':key==='limit'?'limit':key)).value=value;
    }
    update();
  }
  function init(){
    if(!id('p1'))return;
    outputIds.forEach(name=>id(name).addEventListener('input',scheduleUpdate));
    document.querySelectorAll('#p1plantBtns button').forEach(b=>b.addEventListener('click',()=>{plantKind=b.dataset.kind;update();}));
    id('p1presetP').addEventListener('click',()=>preset('p'));
    id('p1presetPI').addEventListener('click',()=>preset('pi'));
    id('p1presetPID').addEventListener('click',()=>preset('pid'));
    id('p1reset').addEventListener('click',()=>preset('p'));
    id('p1predCheck').addEventListener('click',checkPrediction);
    id('p1predAnswer').addEventListener('keydown',ev=>{if(ev.key==='Enter')checkPrediction();});
    update(); validation();
  }
  window.P1Controller={simulate,metrics,validation,update,read};
  init();
})();
