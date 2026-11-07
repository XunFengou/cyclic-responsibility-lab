import {createRedistributionMission} from './redistribution-mission.mjs';

const $=id=>document.getElementById(id);
const plots=globalThis.LabPlots;
const labels={repair:'Cyclic repair',cyclic:'Fixed cyclic',voronoi:'Nearest-k Voronoi',geometricRepair:'Geometric repair'};
const format=(x,digits=1)=>Number.isFinite(x)?x.toFixed(digits):'—';
const percent=x=>format(100*x)+'%';
const color=id=>globalThis.AgentColors?.color(id)||'#126b96';
let dataset,scenario,time=0,playing=false,playFrame,activeRun=null,runToken=0;

function shownTrial(){return scenario?.trials.find(t=>t.method===$('rdPolicy').value)||scenario?.trials[0];}
function atTime(trial){
  let lo=0,hi=trial.frames.length;
  while(lo+1<hi){const mid=(lo+hi)>>1;if(trial.frames[mid].time<=time+1e-8)lo=mid;else hi=mid;}
  return trial.frames[lo];
}
function append(parent,tag,content){const el=document.createElement(tag);el.textContent=content;parent.appendChild(el);return el;}
function metric(parent,name,value){const el=append(parent,'div','');el.className='metric-card';append(el,'span',name);append(el,'strong',value);}
function drawMaps(trial,frame){
  const {c}=plots.prepare('rdMap',560,470),scale=194,center=[280,231],xy=p=>[center[0]+scale*p[0],center[1]-scale*p[1]];
  const circle=(ctx,p,r,fill)=>{ctx.fillStyle=fill;ctx.beginPath();ctx.arc(...p,r,0,2*Math.PI);ctx.fill();};
  circle(c,center,scale,'#eff4f4');
  const {angles,widths}=frame.row,narrow=widths.indexOf(Math.min(...widths));
  for(let i=0;i<angles.length;i++){
    const start=angles[i],end=start+widths[i];
    c.beginPath();c.moveTo(...center);c.arc(...center,scale,-end,-start);c.closePath();
    c.fillStyle=i===narrow?'#e3b25160':color(frame.agents[i].id)+'20';c.fill();
    plots.line(c,[center,xy([Math.cos(start),Math.sin(start)])],color(frame.agents[i].id),.8);
  }
  for(const o of trial.obstacles)circle(c,xy([o.x,o.y]),o.r*scale,'#506270');
  const h=trial.hotspot,a=Math.atan2(h.center[1],h.center[0]),r=Math.hypot(...h.center)*scale/10;
  c.strokeStyle='#bc4031';c.lineWidth=10;c.beginPath();c.arc(...center,r,-a-h.angularHalfWidth,-a+h.angularHalfWidth);c.stroke();
  for(const robot of frame.robots){const p=xy(robot.p);circle(c,p,robot.id===frame.workflow.carrierId?7:4.6,color(robot.id));c.fillStyle='#20374a';c.font='11px system-ui';c.fillText(String(robot.id),p[0]+8,p[1]+3);}
  circle(c,xy(frame.object.pos.map(v=>v/10)),6,'#d78a14');
  c.font='12px system-ui';c.fillStyle='#294d61';c.fillText('Gold sector: minimum angular width',22,451);

  const local=plots.prepare('rdLocal',520,470).c,X=x=>260+110*(x+5.2),Y=y=>111+99*(.55-y);
  for(const pad of trial.pads){
    const [x,y]=pad.pos,[hx,hy]=pad.halfSize;
    local.fillStyle=pad.id==='source'?'#d99e82':pad.id==='destination'?'#9bcead':'#a3bdcd';
    local.fillRect(X(x-hx),Y(y+hy),220*hx,198*hy);
    if(pad.id===frame.workflow.padId){local.strokeStyle='#172f42';local.lineWidth=2;local.strokeRect(X(x-hx),Y(y+hy),220*hx,198*hy);}
    local.font='12px system-ui';local.fillStyle='#294d61';local.fillText(pad.id,X(x+hx)+10,Y(y)+4);
  }
  const trail=trial.frames.filter(f=>f.time<=frame.time+1e-8).map(f=>[X(f.object.pos[0]),Y(f.object.pos[1])]);
  plots.line(local,trail,'#c58420',1.7);
  circle(local,[X(frame.object.pos[0]),Y(frame.object.pos[1])],7,'#d78a14');
  for(const robot of frame.robots){
    const x=X(robot.p[0]*10),y=Y(robot.p[1]*10);
    if(x<16||x>500||y<25||y>435)continue;
    circle(local,[x,y],robot.id===frame.workflow.carrierId?13:7,color(robot.id));local.font='12px system-ui';local.fillStyle='#253f53';local.fillText('ID '+robot.id,x+15,y+3);
  }
  local.fillStyle='#294d61';local.fillText('Local view · top down · actual object position',16,445);
}

function drawCharts(trial){
  const series=(name,key,map=x=>x)=>({name,points:trial.frames.map(f=>[f.time,map(f.row[key])])});
  const chart=(id,sets,units)=>plots.chart(id,sets,'Time (s)',units,{cursor:time});
  chart('rdSource',[series('Source patches','sourceLoad',x=>100*x),series('Certified deposit','transferredLoad',x=>100*x)],'Share of total demand (%)');
  const baseline=trial.frames[0].row.imbalance;
  chart('rdBalance',[series('Fixed angles','imbalance'),series('Fixed-pose nearest-k','nearestFixedRobotImbalance'),{name:'Parcel stays at source',dashed:true,points:[[0,baseline],[trial.frames.at(-1).time,baseline]]}],'Relative RMS');
  chart('rdAngle',[series('Current minimum','minimumAngle'),series('Best sampled phase','bestEqualMassMinAngle')],'Angle (degrees)');
  chart('rdForce',[series('Measured normal sum','normalForce'),{name:'4 × command target',points:trial.frames.map(f=>[f.time,4*f.object.forceTarget])}],'Force (N)');
  chart('rdSlip',[series('Tangential speed','slipSpeed',x=>1000*x)],'Speed (mm/s)');
}

function draw(){
  const trial=shownTrial();if(!trial?.frames.length)return;
  const frame=atTime(trial),row=frame.row,workflow=frame.workflow;
  $('rdClock').textContent=format(frame.time)+' s';$('rdTime').value=time;
  $('rdStatus').textContent=`${labels[trial.method]} · ${workflow.stage} · ${workflow.carrierId?`carrier ID ${workflow.carrierId}`:'no reserved carrier'}`;
  const cards=$('rdMetrics');cards.replaceChildren();
  metric(cards,'Source concentration',percent(row.sourceLoad));metric(cards,'Certified relocated demand',percent(row.transferredLoad));
  metric(cards,'Fixed-angle load RMS',format(row.imbalance,3));metric(cards,'Smallest current sector',format(row.minimumAngle)+'°');
  metric(cards,'Measured jaw load',format(row.normalForce)+' N');metric(cards,'Joint-limit excess',format(row.jointLimitExcess,4)+' native units');
  $('rdSupport').textContent=`Selected pad ${workflow.padId||'—'} · actual support ${frame.object.supportId||'airborne or unsupported'} · upward load ${format(frame.object.contact.supportForce)} N`;
  const events=$('rdEvents');events.replaceChildren();
  for(const e of trial.events.filter(e=>e.time<=frame.time+1e-8).slice(-5))append(events,'li',`${format(e.time)} s · ${e.type}${e.pad?' · '+e.pad:''}${e.reason?' · '+e.reason:''}`);
  $('rdAlgebra').textContent=`N = ${row.N} cyclic roles, k = ${row.k}, gcd = ${row.gcd}. There are ${row.gcd} jump orbits and ${row.gcd-1} zero-sum Fourier blind directions. Physical bodies ${row.physicalN}, available robots ${row.available}. Transport redistributes demand; a degraded carrier reduces availability.`;
  drawMaps(trial,frame);drawCharts(trial);
}

function results(){
  const body=$('rdResults');body.replaceChildren();
  for(const trial of scenario.trials){
    const f=trial.frames.at(-1),w=f.workflow,r=append(body,'tr','');
    for(const v of [labels[trial.method],w.history.map(h=>h.id).join(' → ')||'—',w.safeSetdowns.map(s=>s.padId).join(', ')||'—',format(w.placementCertificate?.time),format(w.confirmationCertificate?.time),format(f.row.imbalance,3),w.success?'Confirmed':w.placementCertificate?'Placed, confirmation incomplete':f.workflow.stage])append(r,'td',v);
  }
}
function stopPlay(){playing=false;if(playFrame)cancelAnimationFrame(playFrame);$('rdPlay').textContent='Play replay';}
function selectScenario(next){
  if(!next)return;
  stopPlay();scenario=next;time=0;$('rdTime').max=Math.max(...next.trials.map(t=>t.frames.at(-1).time));
  results();draw();
}
function play(){
  if(!shownTrial())return;
  if(playing){stopPlay();return;}
  const end=Number($('rdTime').max);if(time>=end)time=0;
  playing=true;$('rdPlay').textContent='Pause replay';
  let last=performance.now(),lastDraw=last;
  function tick(now){
    if(!playing)return;
    time=Math.min(end,time+(now-last)*.004);last=now;
    if(now-lastDraw>95||time===end){draw();lastDraw=now;}
    if(time===end)stopPlay();else playFrame=requestAnimationFrame(tick);
  }
  playFrame=requestAnimationFrame(tick);
}

async function run(){
  const token=++runToken;stopPlay();$('rdRun').disabled=true;$('rdStop').disabled=false;
  $('rdRunStatus').textContent='Loading MuJoCo and the articulated robot model…';
  try{
    const [{default:load},response]=await Promise.all([import('./vendor/mujoco/mujoco.js'),fetch('./vendor/tiago-pro/planar20.xml')]);
    if(!response.ok)throw Error('Robot model unavailable');
    const [M,xml]=await Promise.all([load(),response.text()]);
    if(token!==runToken)return;
    const Mission=createRedistributionMission({T:globalThis.TiagoCoverage,E:globalThis.ExecutionPlant,A:globalThis.ExecutionAllocation,C:globalThis.CoverageComparison,L:globalThis.SectorLab,Metrics:globalThis.CoverageLab});
    const mission=new Mission(M,xml,$('rdPolicy').value,{N:+$('rdN').value,k:+$('rdK').value,mass:+$('rdMass').value,friction:+$('rdMu').value,faultMode:$('rdFault').value});
    activeRun=mission;
    const step=()=>{
      if(token!==runToken)return;
      if(mission.step()&&!['complete','quarantined'].includes(mission.workflow.stage)){
        $('rdRunStatus').textContent=`Physical time ${format(mission.time)} s · ${mission.workflow.stage}`;requestAnimationFrame(step);return;
      }
      const record=mission.export();mission.close();activeRun=null;
      $('rdRunStatus').textContent=`Trial ended at ${format(record.frames.at(-1).time)} s${record.reason?' · '+record.reason:''}`;
      $('rdRun').disabled=false;$('rdStop').disabled=true;
      const custom={id:'custom',title:'Most recent custom trial',trials:[{...record,obstacles:record.domain.obstacles}]};
      if(dataset){
        if(!dataset.scenarios.some(s=>s.id==='custom'))append($('rdScenario'),'option',custom.title).value='custom';
        dataset.scenarios=dataset.scenarios.filter(s=>s.id!=='custom').concat(custom);
        $('rdScenario').value='custom';
      }
      selectScenario(custom);
    };
    requestAnimationFrame(step);
  }catch(e){if(token!==runToken)return;$('rdRunStatus').textContent=e.message;$('rdRun').disabled=false;$('rdStop').disabled=true;}
}

$('rdPlay').onclick=play;
$('rdTime').oninput=()=>{stopPlay();time=+$('rdTime').value;draw();};
$('rdBefore').onclick=()=>{stopPlay();time=0;draw();};
$('rdAfter').onclick=()=>{stopPlay();time=shownTrial()?.frames.at(-1).time||0;draw();};
$('rdPolicy').onchange=draw;
$('rdScenario').onchange=()=>selectScenario(dataset.scenarios.find(s=>s.id===$('rdScenario').value));
$('rdRun').onclick=run;
$('rdStop').onclick=()=>{runToken++;activeRun?.close();activeRun=null;$('rdRunStatus').textContent='Trial stopped.';$('rdRun').disabled=false;$('rdStop').disabled=true;};
try{
  const response=await fetch('result/redistribution-recordings.json');
  if(!response.ok)throw Error('Comparison recordings could not be loaded');
  dataset=await response.json();
  for(const s of dataset.scenarios){const option=append($('rdScenario'),'option',s.title);option.value=s.id;}
  selectScenario(dataset.scenarios[0]);
}catch(e){$('rdStatus').textContent=e.message;}
