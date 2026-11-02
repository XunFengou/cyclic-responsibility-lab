'use strict';
const fs=require('fs'),path=require('path'),assert=require('assert');
const {Simulation,Geometry,gcd}=require('../src/engine'),Motion=require('../src/dynamics');
const root=path.resolve(__dirname,'..'),out=path.join(root,'test-results');fs.mkdirSync(out,{recursive:true});
const groups=[],runs=[],models=Motion.MODELS;
const base={N:12,serviceOrder:3,geometry:'disk',faultTime:null,duration:35,placement:'scattered',freezeBoundaries:true,resolution:2048};
function run(cfg){const s=new Simulation(cfg);s.record();s.run();return s;}
function test(name,fn){const t=Date.now();fn();groups.push({name,passed:true,milliseconds:Date.now()-t});console.log('PASS',name);}
let fixed={};
test('Six native plant models: frozen-partition cost descent and actual input traces',()=>{
 for(const robotModel of models){const s=run({...base,robotModel});fixed[robotModel]=s;
  assert(s.history.at(-1).targetError<2e-7,robotModel+' did not reach the numerical target tolerance.');
  assert(Math.max(...s.history.map(h=>h.motionCostChange))<2e-12,robotModel+' increased fixed-partition motion cost.');
  assert(s.history.at(-1).costPerOrder<s.history[0].costPerOrder*.9);assert(s.inputEffort>0);assert(s.robotInputs.size===base.N);
  assert(s.maxOperatorError<1e-12);assert(s.maxWindowError<1e-12);
  runs.push({name:'fixed_'+robotModel,robotModel,status:s.status,...s.history.at(-1)});
 }
 for(const model of models)assert.deepEqual(fixed[model].snapshots[0].positions,fixed.single.snapshots[0].positions);
});
test('Orders 1 to 5 match the exact critically damped Cartesian solution',()=>{
 for(let r=1;r<=5;r++){
  const robotModel=r===1?'single':r===2?'double':'high',s=run({...base,robotModel,integratorOrder:Math.max(3,r),duration:3.2});
  const state=s.observe(),initial=s.snapshots[0].positions,a=s.cfg.robotGain,t=s.time;let poly=0,term=1;
  for(let j=0;j<r;j++){if(j)term*=a*t/j;poly+=term;}const response=Math.exp(-a*t)*poly;
  for(let i=0;i<s.roles.length;i++)for(let j=0;j<2;j++)assert(Math.abs(s.roles[i].p[j]-state.centroids[i][j]-(initial[i][j]-state.centroids[i][j])*response)<2e-8);
 }
});
test('Euler-Lagrange inertia, Christoffel identity, computed torque, and holding force',()=>{
 const cfg=fixed.lagrangian.cfg,geo=fixed.lagrangian.geo,y=[.3,-.4,.6,-.2],target=[-.1,.5],m=Motion.mechanics(y,y.slice(2),cfg),d=Motion.derivative(y,target,cfg,geo),a=cfg.robotGain;
 for(let j=0;j<2;j++)assert(Math.abs(d.dy[j+2]-(a*a*(target[j]-y[j])-2*a*y[j+2]))<2e-14);
 assert(m.M.every(v=>v>=cfg.elMass*(1-cfg.elEccentricity)));
 const C=[[.5*m.A*y[3],.5*(m.A*y[2]-m.B*y[3])],[.5*(m.B*y[3]-m.A*y[2]),.5*m.B*y[2]]],Md=[[m.A*y[3],0],[0,m.B*y[2]]];
 for(let i=0;i<2;i++)for(let j=0;j<2;j++)assert(Math.abs((Md[i][j]-2*C[i][j])+(Md[j][i]-2*C[j][i]))<1e-14);
 for(let i=0;i<base.N;i++)assert(Math.hypot(...fixed.double.roles[i].p.map((p,j)=>p-fixed.lagrangian.roles[i].p[j]))<1e-12);
 assert(fixed.lagrangian.history.at(-1).robotInputRms>.01);assert(fixed.double.history.at(-1).robotInputRms<1e-8);
});
test('No-slip constraint, speed/turn limits, wheel mapping, and projected cost derivative',()=>{
 for(const model of ['unicycle','differential']){
  const s=fixed[model],c=s.cfg;assert(s.maxNoSlipResidual<1e-13);
  for(let i=0;i<50;i++){
   const y=[.1*Math.sin(i),.2*Math.cos(i),i*.31],target=[.4,-.3],d=Motion.derivative(y,target,c,s.geo),costDot=(y[0]-target[0])*d.velocity[0]+(y[1]-target[1])*d.velocity[1];
   assert(costDot<1e-14);assert(Math.abs(d.slip)<1e-14);
   if(model==='differential'){assert(d.input.every(u=>Math.abs(u)<=c.wheelSpeedLimit+1e-12));const v=c.wheelRadius*(d.input[0]+d.input[1])/2,w=c.wheelRadius*(d.input[1]-d.input[0])/c.wheelBase;assert(Math.abs(v*Math.cos(y[2])-d.dy[0])<1e-14);assert(Math.abs(w-d.dy[2])<1e-14);}
   else {assert(Math.abs(d.input[0])<=c.speedLimit);assert(Math.abs(d.input[1])<=c.turnLimit);}
  }
 }
});
test('Algebra and packet control are unchanged by the physical dynamics',()=>{
 let reference;
 for(const robotModel of models){const s=run({...base,N:18,serviceOrder:5,robotModel,freezeBoundaries:false,massMode:'random',duration:12});
  if(!reference)reference=s.observe().mass;
  assert(Math.max(...s.observe().mass.map((v,i)=>Math.abs(v-reference[i])))<1e-14);
  assert(s.maxOperatorError<1e-12);assert(s.maxWindowError<1e-12);assert(s.minMassEver>0);
  if(['single','double','high','lagrangian'].includes(robotModel))assert(s.minMarginEver>-1e-9);
 }
});
test('Failure reconstruction and complementary control retain the predicted quotient',()=>{
 const s=run({N:50,serviceOrder:7,robotModel:'double',geometry:'disk',duration:200,policy:'single'});
 const c=run({N:50,serviceOrder:7,robotModel:'double',geometry:'disk',duration:200,policy:'complement'});
 assert(Math.abs(s.history.at(-1).massError-Math.sqrt(6)/50)<1e-11);assert(s.history.at(-1).loadError<1e-12);
 assert(c.history.at(-1).massError<1e-10);assert(c.history.at(-1).targetError<1e-6);assert(c.maxReconstructionError<1e-14);
 assert(gcd(999,21)===3&&gcd(gcd(1008,21),14)===7&&gcd(gcd(gcd(1008,21),14),6)===1);
 runs.push({name:'failure_double_single',robotModel:'double',status:s.status,...s.history.at(-1)},{name:'failure_double_complement',robotModel:'double',status:c.status,...c.history.at(-1)});
});
test('Coverage cost decomposition adds to the actual total change',()=>{
 const s=run({...base,robotModel:'high',freezeBoundaries:false,massMode:'random',duration:15});
 for(let i=1;i<s.history.length;i++){const h=s.history[i],prev=s.history[i-1];assert(Math.abs(h.costPerOrder-prev.costPerOrder-h.motionCostChange-h.boundaryCostChange)<1e-12);}
});
test('Role transfer preserves physical inertial state and geometry',()=>{
 const s=new Simulation({...base,robotModel:'double',freezeBoundaries:false});for(let i=0;i<10;i++)s.step();
 s.network.setup(s.agents,s.geo,s.time,s.epoch);const states=new Map(s.agents.map(a=>[a.id,a.motion.y.slice()])),boundaries=s.roles.map(a=>a.x);s.arrange();
 for(const a of s.agents)assert.deepEqual(a.motion.y,states.get(a.id));assert.deepEqual(s.roles.map(a=>a.x),boundaries);
});
test('Disconnected carrier blocks every model before state evolution',()=>{
 for(const robotModel of models){const s=run({...base,robotModel,carrier:'range',radioRange:.001});assert(s.status==='blocked');assert(s.time===0);}
});
test('Containment limits are exposed: no Cartesian or nonholonomic endpoint projection',()=>{
 const s=new Simulation({...base,robotModel:'unicycle'}),robot=s.agents[0];robot.p=[.9999,0];Motion.initialize(robot,s.cfg);robot.motion.y=[.9999,0,Math.acos(.1)];const move=Motion.advance(robot,[.4,.5],s.cfg,s.geo,.02);
 assert(move.minimumSampledMargin<0);assert(Math.hypot(...robot.p)>1);
 const geo=new Geometry('annulus',2048),cart=new Simulation({...base,robotModel:'double',geometry:'annulus'}),a=cart.agents[0];a.p=[1.5,0];Motion.initialize(a,cart.cfg);const cross=Motion.advance(a,[-1.5,0],cart.cfg,geo,4);assert(cross.minimumSampledMargin<0);
 const safe=new Simulation({...base,robotModel:'single',geometry:'annulus'}),b=safe.agents[0];b.p=[1.5,0];b.nav=geo.coordinates(b.p);b.goalAngle=null;Motion.initialize(b,safe.cfg);const arc=Motion.advance(b,[-1.5,0],safe.cfg,geo,4);assert(arc.minimumSampledMargin>.024999);
});
const report={status:'passed',completedAt:new Date().toISOString(),groups,modelChoices:models.length,integratorOrdersTested:[1,2,3,4,5],maxPacketOperatorError:Math.max(...Object.values(fixed).map(s=>s.maxOperatorError)),maxNoSlipResidual:Math.max(...Object.values(fixed).map(s=>s.maxNoSlipResidual)),
 assumptions:['Shared known demand map and complete reliable packet snapshots.','Motion-time dynamics; communication latency clock is accounting only.','Unicycle numerical goal deadband: 1e-8.','Euler-Lagrange model and compensation are exact and fully actuated.','No pairwise collision avoidance or general annulus guarantee for Cartesian plants.']};
fs.writeFileSync(path.join(out,'dynamics_checks.json'),JSON.stringify(report,null,2));
const keys=Object.keys(runs[0]);fs.writeFileSync(path.join(out,'dynamics_case_summary.csv'),keys.join(',')+'\n'+runs.map(r=>keys.map(k=>JSON.stringify(r[k]??'')).join(',')).join('\n'));
console.log('Completed',groups.length,'scientific validation groups.');
