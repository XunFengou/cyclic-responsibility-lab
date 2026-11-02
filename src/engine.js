/* English, offline, synchronous message-passing reference implementation.
 * Controllers receive local atom measurements and routed packets only.
 * observe() and auditStep() are read-only simulator diagnostics.
 */
(function(root,factory){const api=factory(typeof module==='object'&&module.exports?require('./dynamics.js'):root.SectorDynamics);if(typeof module==='object'&&module.exports)module.exports=api;else root.SectorLab=api;})(typeof globalThis!=='undefined'?globalThis:this,function(Motion){
'use strict';
const TAU=2*Math.PI;
const mod=(x,n)=>((x%n)+n)%n;
function gcd(a,b){while(b){const t=a%b;a=b;b=t;}return a;}
function assert(ok,message){if(!ok)throw new Error(message);}
function rng(seed){let s=seed>>>0;return ()=>{s=(1664525*s+1013904223)>>>0;return (s+.5)/4294967296;};}
function sum(a){let r=0;for(const x of a)r+=x;return r;}
function windowSum(a,k){const n=a.length;return a.map((_,i)=>{let v=0;for(let j=0;j<k;j++)v+=a[mod(i-j,n)];return v;});}
function orbitMean(a,d){const r=Array(d).fill(0),q=a.length/d;for(let i=0;i<a.length;i++)r[i%d]+=a[i]/q;return a.map((_,i)=>r[i%d]);}
function relError(a,mean){return Math.sqrt(sum(a.map(x=>(x-mean)**2))/a.length)/mean;}
function serviceCost(v,p){return .5*(v[3]-2*(p[0]*v[1]+p[1]*v[2])+v[0]*(p[0]**2+p[1]**2));}

class Geometry{
 constructor(mode='annulus',resolution=8192){
  this.mode=mode;this.resolution=resolution;this.h=TAU/resolution;this.margin=mode==='annulus'?.025:0;
  this.values=[];this.cumulative=[Array(4).fill(0)];
  for(let i=0;i<=resolution;i++){
   const a=i*this.h;let v;
   if(mode==='disk')v=[.5,Math.cos(a)/3,Math.sin(a)/3,.25];
   else{
    const [ri,ro]=this.bounds(a),A=1+.45*Math.cos(3*a-.4)+.25*Math.sin(2*a),b=.1;
    const w=A*(ro**2-ri**2)/2+b*(ro**3-ri**3)/3;
    const first=A*(ro**3-ri**3)/3+b*(ro**4-ri**4)/4;
    v=[w,first*Math.cos(a),first*Math.sin(a),A*(ro**4-ri**4)/4+b*(ro**5-ri**5)/5];
   }
   this.values.push(v);
   if(i)this.cumulative.push(v.map((x,h)=>this.cumulative[i-1][h]+this.h*(x+this.values[i-1][h])/2));
  }
  this.rawMass=this.cumulative[resolution][0];
  for(let i=0;i<=resolution;i++)for(let j=0;j<4;j++){this.values[i][j]/=this.rawMass;this.cumulative[i][j]/=this.rawMass;}
  this.cumulative[resolution][0]=1;this.total=this.cumulative[resolution].slice();
 }
 bounds(a){return this.mode==='disk'?[0,1]:[1+.12*Math.cos(2*a)+.06*Math.sin(5*a),2+.20*Math.sin(3*a)+.10*Math.cos(a)];}
 primitive(a){
  const turns=Math.floor(a/TAU),y=a-turns*TAU,i=Math.min(this.resolution-1,Math.floor(y/this.h)),t=y-i*this.h;
  return this.total.map((M,j)=>turns*M+this.cumulative[i][j]+t*this.values[i][j]+t*t*(this.values[i+1][j]-this.values[i][j])/(2*this.h));
 }
 inverse(x){
  const turns=Math.floor(x),y=x-turns;let lo=0,hi=this.resolution;
  while(hi-lo>1){const mid=(lo+hi)>>1;if(this.cumulative[mid][0]<=y)lo=mid;else hi=mid;}
  const offset=y-this.cumulative[lo][0],w=this.values[lo][0],s=(this.values[lo+1][0]-w)/this.h;
  const distance=offset===0?0:2*offset/(w+Math.sqrt(Math.max(0,w*w+2*s*offset)));
  return turns*TAU+lo*this.h+distance;
 }
 atom(left,right){
  const a=this.primitive(this.inverse(left)),b=this.primitive(this.inverse(right));
  return [right-left,b[1]-a[1],b[2]-a[2],b[3]-a[3]];
 }
 coordinates(p){const a=Math.atan2(p[1],p[0]),r=Math.hypot(...p),[ri,ro]=this.bounds(a);return [a,(r-ri-this.margin)/(ro-ri-2*this.margin)];}
 map(z){const [ri,ro]=this.bounds(z[0]),r=ri+this.margin+z[1]*(ro-ri-2*this.margin);return [r*Math.cos(z[0]),r*Math.sin(z[0])];}
 feasibleTarget(c){
  if(this.mode==='disk')return c.slice();
  const a=Math.atan2(c[1],c[0]),[ri,ro]=this.bounds(a),r=Math.max(ri+this.margin,Math.min(ro-this.margin,Math.hypot(...c)));
  return [r*Math.cos(a),r*Math.sin(a)];
 }
 marginAt(p){const [ri,ro]=this.bounds(Math.atan2(p[1],p[0])),r=Math.hypot(...p);return Math.min(r-ri,ro-r);}
 lineOfSight(a,b){
  if(this.mode==='disk')return true;
  const dx=b[0]-a[0],dy=b[1]-a[1],norm2=dx*dx+dy*dy;
  if(norm2<1e-24)return this.marginAt(a)>0;
  const t=Math.max(0,Math.min(1,-(a[0]*dx+a[1]*dy)/norm2));
  const rmin=Math.hypot(a[0]+t*dx,a[1]+t*dy);if(rmin<.82)return false;
  const length=Math.sqrt(norm2),L=length*(1+.7/rmin),J=16;
  let margin=Infinity;for(let h=0;h<=J;h++)margin=Math.min(margin,this.marginAt([a[0]+h*dx/J,a[1]+h*dy/J]));
  return margin>L/(2*J)+1e-12;
 }
}

class Network{
 constructor(cfg){this.cfg=cfg;this.total={packetHops:0,payloadByteHops:0,totalByteHops:0,rounds:0,topologyPacketHops:0};this.trace=[];this.traceCounts={};this.revisions=0;this.lastKey='';}
 setup(agents,geometry,motionTime,epoch){
  this.agents=agents;this.index=new Map(agents.map((a,i)=>[a.id,i]));this.n=agents.length;this.epoch=epoch;
  const n=this.n;this.adj=Array.from({length:n},()=>[]);let edges=0;
  const add=(i,j)=>{if(i!==j&&!this.adj[i].includes(j)){this.adj[i].push(j);this.adj[j].push(i);edges++;}};
  if(this.cfg.carrier==='range'){
   for(let i=0;i<n;i++)for(let j=0;j<i;j++)if(Math.hypot(agents[i].p[0]-agents[j].p[0],agents[i].p[1]-agents[j].p[1])<=this.cfg.radioRange&&(!this.cfg.lineOfSight||geometry.lineOfSight(agents[i].p,agents[j].p)))add(i,j);
  }else{
   for(let i=0;i<n;i++)if(!(this.cfg.cutTime!==null&&motionTime>=this.cfg.cutTime&&i===n-1))add(i,(i+1)%n);
  }
  for(const row of this.adj)row.sort((a,b)=>a-b);
  const key=agents.map(a=>a.id).join(',')+'|'+this.adj.map(a=>a.join(',')).join(';');
  this.changed=key!==this.lastKey;this.edgeCount=edges;
  const reached=new Set([0]),queue=[0];
  for(let h=0;h<queue.length;h++)for(const v of this.adj[queue[h]])if(!reached.has(v)){reached.add(v);queue.push(v);}
  if(reached.size!==n){this.connected=false;this.lastKey='';return false;}
  this.connected=true;
  // Static ring/path routes are cached between announced changes. A metric
  // radio graph runs a full, deliberately simple link-state flood each epoch.
  if(this.changed||this.cfg.carrier==='range'){
   this.revisions+=Number(this.changed);this.discover();
  }
  if(this.changed||!this.paths){
   this.paths=[];
   for(let s=0;s<n;s++){
    const map=this.agents[s].routingMap,parent=Array(n).fill(-1),localQueue=[s];parent[s]=s;
    for(let h=0;h<localQueue.length;h++)for(const v of map[localQueue[h]])if(parent[v]<0){parent[v]=localQueue[h];localQueue.push(v);}
    assert(localQueue.length===n,'A source could not construct routes from its received map.');
    this.paths[s]=Array.from({length:n},(_,d)=>{const path=[d];while(path[path.length-1]!==s)path.push(parent[path[path.length-1]]);return path.reverse();});
   }
  }
  this.lastKey=key;return true;
 }
 charge(type,from,to,payloadBytes,round,data=null){
  assert(this.adj[from].includes(to),'A packet attempted a nonexistent carrier edge.');
  this.total.packetHops++;this.total.payloadByteHops+=payloadBytes;this.total.totalByteHops+=payloadBytes+24;
  if(type==='link-state')this.total.topologyPacketHops++;
  this.traceCounts[type]=(this.traceCounts[type]||0)+1;
  if(this.traceCounts[type]<=24&&this.trace.length<200)this.trace.push({epoch:this.epoch,type,from:this.agents[from].id,to:this.agents[to].id,payloadBytes,round,data});
 }
 discover(){
  const n=this.n;const maps=Array.from({length:n},()=>Array(n));
  // N source slots of N rounds. A receiver forwards a source advertisement
  // only once. This is decentralized all-gather, not a global state input.
  for(let s=0;s<n;s++){
   const seen=Array(n).fill(false);let front=[s];seen[s]=true;maps[s][s]=this.adj[s].slice();
   for(let r=0;r<n;r++){
    const next=[];for(const u of front)for(const v of this.adj[u]){
     this.charge('link-state',u,v,4*this.adj[s].length,s*n+r,{origin:this.agents[s].id,neighbors:this.adj[s].map(h=>this.agents[h].id)});
     if(!seen[v]){seen[v]=true;maps[v][s]=this.adj[s].slice();next.push(v);}
    }front=next;
   }
  }
  for(let i=0;i<n;i++){assert(maps[i].filter(Boolean).length===n,'Incomplete local topology map.');this.agents[i].routingMap=maps[i];}
  this.total.rounds+=n*n;
 }
 batch(packets,type,payloadBytes){
  let pending=packets.map(p=>({...p,data:p.data.slice(),path:this.paths[this.index.get(p.from.id)][this.index.get(p.to.id)],hop:0}));
  let round=0;
  while(pending.length){
   assert(round<100000,'Routing queue failed to terminate.');const used=new Set(),next=[];
   for(const p of pending){
    assert(p.epoch===this.epoch,'A stale packet entered the current epoch.');
    if(p.hop===p.path.length-1){p.receive(p.data);continue;}
    const u=p.path[p.hop],v=p.path[p.hop+1],edge=u*this.n+v;
    if(used.has(edge)){next.push(p);continue;}used.add(edge);
    this.charge(type,u,v,payloadBytes,round,{origin:p.from.id,destination:p.to.id,values:p.data.slice()});p.hop++;
    if(p.hop===p.path.length-1)p.receive(p.data);else next.push(p);
   }
   pending=next;round++;
  }
  this.total.rounds+=round;return round;
 }
 allGatherAngles(){
  const n=this.n,received=Array.from({length:n},()=>Array(n));
  for(let source=0;source<n;source++){
   const angle=Math.atan2(this.agents[source].p[1],this.agents[source].p[0]);
   const seen=Array(n).fill(false);let front=[source];seen[source]=true;received[source][source]=angle;
   for(let round=0;round<n;round++){
    const next=[];for(const u of front)for(const v of this.adj[u]){
     this.charge('arrangement-angle',u,v,8,source*n+round,{origin:this.agents[source].id,angle});
     if(!seen[v]){seen[v]=true;received[v][source]=angle;next.push(v);}
    }front=next;
   }
  }
  this.total.rounds+=n*n;
  const ids=this.agents.map(a=>a.id),orders=received.map(values=>values.map((angle,i)=>({angle,id:ids[i]})).sort((a,b)=>a.angle-b.angle||a.id-b.id).map(a=>a.id));
  assert(orders.every(a=>a.join(',')===orders[0].join(',')),'Agents disagreed on the new arrangement.');
  return orders[0];
 }
}

class Simulation{
 constructor(options={}){
  this.cfg=Object.assign({N:50,serviceOrder:7,policy:'single',geometry:'annulus',carrier:'ring',radioRange:.65,lineOfSight:true,
   dt:.2,gain:1,robotGain:.8,roundSeconds:.0001,duration:240,faultTime:40,failedId:13,extraIdle:false,
   secondFaultTime:null,secondFailedId:33,cutTime:null,reassignTime:null,placement:'near',massMode:'uniform',seed:2718,audit:true,resolution:8192,
   robotModel:'single',integratorOrder:3,robotStep:.01,headingGain:1.6,speedLimit:.9,turnLimit:2,wheelRadius:.08,wheelBase:.3,wheelSpeedLimit:12,
   elMass:1.5,elEccentricity:.25,elDamping:.15,elGravity:.3,freezeBoundaries:false},options);
  const c=this.cfg;assert(Number.isInteger(c.N)&&Number.isInteger(c.serviceOrder)&&c.N>=2&&c.serviceOrder>=1&&c.serviceOrder<=c.N,'Invalid N or service order.');
  assert(Number.isFinite(c.duration)&&c.duration>0&&Number.isFinite(c.roundSeconds)&&c.roundSeconds>=0&&Number.isFinite(c.robotGain)&&c.robotGain>0,'Invalid duration, round time, or robot gain.');
  assert(c.geometry==='disk'||c.geometry==='annulus','Unknown workspace geometry.');
  Motion.validate(c);
  assert(c.dt*c.gain>0&&c.dt*c.gain<=.5,'Positivity requires 0 < dt * gain <= 0.5.');
  this.geo=new Geometry(c.geometry,c.resolution);this.network=new Network(c);this.random=rng(c.seed);
  this.agents=[];this.roles=[];this.time=0;this.wallTime=0;this.epoch=0;this.history=[];this.events=[];this.snapshots=[];this.status='ready';
  this.maxOperatorError=0;this.maxWindowError=0;this.maxMassRealizationError=0;this.maxReconstructionError=0;this.faultDone=false;this.secondDone=false;this.reassigned=false;
  let masses=Array.from({length:c.N},(_,i)=>c.massMode==='uniform'?1:Math.exp(.4*Math.sin(TAU*i/c.N)+.4*(this.random()-.5)));
  masses=masses.map(x=>x/sum(masses));let x=.03;
  for(let i=0;i<c.N;i++){
   const a={id:i+1,x,p:[0,0],goalAngle:null,local:null};x+=masses[i];this.agents.push(a);this.roles.push(a);
  }
  const initial=this.observe();
  for(let i=0;i<c.N;i++){
   const target=this.geo.feasibleTarget(initial.centroids[i]);let p;
   if(c.geometry==='disk'){
    const angle=c.placement==='scattered'?TAU*this.random():Math.atan2(target[1],target[0])+.25*Math.sin(i+1);
    const r=c.placement==='scattered'?.2+.65*this.random():Math.min(.95,Math.hypot(...target)+.08*Math.cos(i));p=[r*Math.cos(angle),r*Math.sin(angle)];
   }else{
    const z=this.geo.coordinates(target);z[0]=c.placement==='scattered'?TAU*this.random():z[0]+.13*Math.sin(1.3*i);z[1]=c.placement==='scattered'?.2+.6*this.random():Math.max(.1,Math.min(.9,z[1]+.1*Math.cos(i)));p=this.geo.map(z);
   }
   this.agents[i].p=p;this.agents[i].nav=c.geometry==='annulus'?this.geo.coordinates(p):null;
  }
  this.trajectories=new Map(this.agents.map(a=>[a.id,[a.p.slice()]]));
  for(const a of this.agents)Motion.initialize(a,c);
  this.robotInputs=new Map(this.agents.map(a=>[a.id,[{motionTime:0,input:[0,0],speed:0}]]));
  this.inputEffort=0;this.inputPeak=0;this.maxNoSlipResidual=0;this.extent=Math.max(...this.agents.flatMap(a=>a.p.map(Math.abs)));
  this.lastEpoch={inputRms:0,inputMax:0,boundaryRms:0,boundaryAngleRms:0,motionCostBefore:null,motionCostAfter:null};
  this.initialPhase=sum(this.roles.map(a=>a.x));this.phaseAnchor=this.initialPhase;this.minMassEver=Infinity;this.minMarginEver=Infinity;
 }
 controlOrders(){
  const k=this.cfg.serviceOrder;
  if(!this.faultDone||this.cfg.policy==='single')return [k];
  if(this.cfg.policy==='basic')return [1];
  if(this.cfg.policy==='complement')return k<this.roles.length?[k,k+1]:[k,k-1].filter(x=>x>=1);
  if(this.cfg.policy==='switch')return [Math.min(this.roles.length,k+Math.floor((this.time-this.cfg.faultTime+1e-9)/4)%2)];
  return [k];
 }
 dispatch(type,items,width){this.network.batch(items.map(x=>({...x,epoch:this.epoch})),type,width);}
 protocol(){
  const n=this.roles.length,k=this.cfg.serviceOrder,orders=this.controlOrders(),maxOrder=Math.max(k,...orders),maxControl=Math.max(...orders);
  const role=this.roles;
  for(const a of role)a.local={nextX:null,aggregate:null,relay:null,seen:new Set(),loads:Array(orders.length).fill(null),incoming:Array(orders.length).fill(null),previous:null,service:null};
  this.dispatch('next-boundary',role.map((a,i)=>({from:a,to:role[mod(i-1,n)],data:[a.x],receive:data=>{role[mod(i-1,n)].local.nextX=data[0];}})),8);
  for(let i=0;i<n;i++){
   const a=role[i],right=a.local.nextX+(i===n-1?1:0),atom=this.geo.atom(a.x,right),s=a.local;
   assert(atom[0]>0,'An atom is empty.');s.atom=atom;s.aggregate=atom.slice();s.relay=atom.slice();s.seen.add(a.id);
   if(k===1)s.service=atom.slice();for(let h=0;h<orders.length;h++)if(orders[h]===1){s.loads[h]=atom[0];s.incoming[h]=atom[0];}
  }
  for(let distance=1;distance<maxOrder;distance++){
   const incoming=Array(n);
   this.dispatch('atom-moment',role.map((a,i)=>({from:a,to:role[(i+1)%n],data:a.local.relay,receive:data=>{incoming[(i+1)%n]=data;}})),32);
   for(let i=0;i<n;i++){
    const s=role[i].local,origin=role[mod(i-distance,n)].id;assert(!s.seen.has(origin),'An atom token was counted twice.');s.seen.add(origin);
    s.relay=incoming[i];for(let j=0;j<4;j++)s.aggregate[j]+=incoming[i][j];
    if(distance+1===k)s.service=s.aggregate.slice();for(let h=0;h<orders.length;h++)if(distance+1===orders[h])s.loads[h]=s.aggregate[0];
   }
  }
  this.dispatch('previous-load',role.map((a,i)=>({from:a,to:role[(i+1)%n],data:a.local.loads,receive:data=>{role[(i+1)%n].local.previous=data;}})),8*orders.length);
  let relay=role.map(a=>a.local.loads.slice());
  for(let distance=1;distance<maxControl;distance++){
   const incoming=Array(n);
   this.dispatch('future-load',role.map((a,i)=>({from:a,to:role[mod(i-1,n)],data:relay[i],receive:data=>{incoming[mod(i-1,n)]=data;}})),8*orders.length);
   relay=incoming;
   for(let i=0;i<n;i++)for(let h=0;h<orders.length;h++)if(distance===orders[h]-1)role[i].local.incoming[h]=incoming[i][h];
  }
  for(const a of role)assert(a.local.service&&a.local.incoming.every(Number.isFinite)&&a.local.previous.every(Number.isFinite),'A controller is missing required packets.');
  return orders;
 }
 remove(ids,label){
  assert(ids.length>0&&new Set(ids).size===ids.length&&ids.every(id=>this.agents.some(a=>a.id===id)),'A failure event must name distinct active robot IDs.');
  const previous=this.roles.slice(),oldX=previous.map(a=>a.x),oldMass=oldX.map((x,i)=>(i===previous.length-1?oldX[0]+1:oldX[i+1])-x);
  const keep=previous.map((a,i)=>ids.includes(a.id)?-1:i).filter(i=>i>=0);
  assert(keep.length>=this.cfg.serviceOrder,'The fault leaves fewer robots than the service order.');
  const before=new Map(previous.map(a=>[a.id,a.p.slice()]));this.roles=keep.map(i=>previous[i]);this.agents=this.agents.filter(a=>!ids.includes(a.id));
  const after=this.observe();let error=0;
  for(let j=0;j<keep.length;j++){
   let stop=keep[(j+1)%keep.length];if(stop<=keep[j])stop+=previous.length;let merged=0;
   for(let t=keep[j];t<stop;t++)merged+=oldMass[t%previous.length];error=Math.max(error,Math.abs(merged-after.mass[j]));
   assert(this.roles[j].p.every((v,h)=>v===before.get(this.roles[j].id)[h]),'A survivor position jumped.');
  }
  this.maxReconstructionError=Math.max(this.maxReconstructionError,error);this.phaseAnchor=sum(this.roles.map(a=>a.x));this.epoch++;
  this.events.push({type:label,motionTime:this.time,wallTime:this.wallTime,removed:ids.slice(),beforeN:previous.length,afterN:this.roles.length,gcd:gcd(this.roles.length,this.cfg.serviceOrder),mergeError:error});
 }
 arrange(){
  const old=this.roles.slice(),x=old.map(a=>a.x),beforePositions=new Map(this.agents.map(a=>[a.id,a.p.slice()])),ids=this.network.allGatherAngles(),byId=new Map(this.agents.map(a=>[a.id,a])),next=ids.map(id=>byId.get(id));
  this.dispatch('role-state-transfer',old.map((a,i)=>({from:a,to:next[i],data:[x[i]],receive:data=>{next[i].newX=data[0];}})),8);
  for(const a of next){a.x=a.newX;delete a.newX;a.goalAngle=null;}this.roles=next;this.epoch++;
  const positionJump=Math.max(...this.agents.map(a=>Math.hypot(a.p[0]-beforePositions.get(a.id)[0],a.p[1]-beforePositions.get(a.id)[1])));
  const boundaryJump=Math.max(...next.map((a,i)=>Math.abs(a.x-x[i])));
  assert(positionJump===0&&boundaryJump===0,'An atomic role transfer changed geometry or robot positions.');
  this.events.push({type:'angle-ranked role reassignment',motionTime:this.time,wallTime:this.wallTime,roleOwners:ids,positionJump,boundaryJump});this.reassigned=true;
 }
 step(){
  if(this.status==='blocked'||this.status==='completed')return false;
  if(this.time>=this.cfg.duration-1e-10){this.status='completed';return false;}
  if(!this.faultDone&&this.cfg.faultTime!==null&&this.time>=this.cfg.faultTime-1e-10){this.remove(this.cfg.extraIdle?[this.cfg.failedId,this.cfg.failedId+1]:[this.cfg.failedId],'robot failure');this.faultDone=true;}
  if(!this.secondDone&&this.cfg.secondFaultTime!==null&&this.time>=this.cfg.secondFaultTime-1e-10){this.remove([this.cfg.secondFailedId],'second robot failure');this.secondDone=true;}
  const oldTotal={...this.network.total};
  if(!this.network.setup(this.agents,this.geo,this.time,this.epoch)){
   this.status='blocked';this.blockReason='The carrier graph is disconnected. This epoch has no complete distributed snapshot.';this.record();return false;
  }
  if(this.network.changed)this.events.push({type:'carrier topology changed',motionTime:this.time,wallTime:this.wallTime,edges:this.network.edgeCount,revision:this.network.revisions});
  if(!this.reassigned&&this.cfg.reassignTime!==null&&this.time>=this.cfg.reassignTime-1e-10)this.arrange();
  this.network.epoch=this.epoch;
  const orders=this.protocol(),n=this.roles.length,weight=1/orders.length,beta=this.cfg.freezeBoundaries?0:this.cfg.dt*this.cfg.gain;
  const oldMass=this.roles.map(a=>a.local.atom[0]),oldX=this.roles.map(a=>a.x);
  const newX=this.roles.map(a=>a.x+beta*weight*sum(a.local.incoming.map((v,h)=>v-a.local.previous[h])));
  const commRounds=this.network.total.rounds-oldTotal.rounds;this.wallTime+=commRounds*this.cfg.roundSeconds;
  let motionCostBefore=0,motionCostAfter=0,inputIntegral=0,inputPeak=0;
  for(let i=0;i<n;i++){
   const a=this.roles[i],service=a.local.service,centroid=[service[1]/service[0],service[2]/service[0]],goal=this.geo.feasibleTarget(centroid);
   motionCostBefore+=serviceCost(service,a.p)/this.cfg.serviceOrder;
   a.x=newX[i];a.lastGoal=goal;a.lastCentroid=centroid;
   const movement=Motion.advance(a,goal,this.cfg,this.geo,this.cfg.dt);motionCostAfter+=serviceCost(service,a.p)/this.cfg.serviceOrder;
   inputIntegral+=movement.normalizedIntegral;inputPeak=Math.max(inputPeak,movement.peak);this.maxNoSlipResidual=Math.max(this.maxNoSlipResidual,movement.maxSlip);
   this.minMarginEver=Math.min(this.minMarginEver,movement.minimumSampledMargin);this.extent=Math.max(this.extent,...a.p.map(Math.abs));
   const path=this.trajectories.get(a.id);for(const p of movement.points)if(Math.hypot(p[0]-path.at(-1)[0],p[1]-path.at(-1)[1])>.01)path.push(p);
   const inputs=this.robotInputs.get(a.id),before={motionTime:this.time,input:movement.beforeInput,speed:Math.hypot(...movement.velocity)};
   if(inputs.length===1&&inputs[0].motionTime===0)inputs[0]=before;
   else if(Math.hypot(...before.input.map((u,j)=>u-inputs.at(-1).input[j]))>1e-8)inputs.push(before);
   inputs.push({motionTime:this.time+this.cfg.dt,input:movement.afterInput,speed:Math.hypot(...movement.velocity)});
  }
  this.inputEffort+=inputIntegral;this.inputPeak=Math.max(this.inputPeak,inputPeak);
  this.lastEpoch={inputRms:Math.sqrt(inputIntegral/(n*this.cfg.dt)),inputMax:inputPeak,motionCostBefore,motionCostAfter,
   boundaryRms:Math.sqrt(sum(newX.map((x,i)=>((x-oldX[i])/this.cfg.dt)**2))/n),
   boundaryAngleRms:Math.sqrt(sum(newX.map((x,i)=>((this.geo.inverse(x)-this.geo.inverse(oldX[i]))/this.cfg.dt)**2))/n)};
  this.time+=this.cfg.dt;this.wallTime+=this.cfg.dt;this.epoch++;
  if(this.cfg.audit){
   const actual=this.roles.map((a,i)=>(i===n-1?newX[0]+1:newX[i+1])-a.x);
   for(let i=0;i<n;i++){
    let expected=oldMass[i];for(const k of orders)expected+=beta*weight*(oldMass[(i+k)%n]+oldMass[mod(i-k,n)]-2*oldMass[i]);
    this.maxOperatorError=Math.max(this.maxOperatorError,Math.abs(actual[i]-expected));
    const local=this.roles[i].local.service,check=Array(4).fill(0);
    for(let j=0;j<this.cfg.serviceOrder;j++)for(let h=0;h<4;h++)check[h]+=this.roles[mod(i-j,n)].local.atom[h];
    this.maxWindowError=Math.max(this.maxWindowError,...local.map((v,h)=>Math.abs(v-check[h])));
   }
   assert(this.maxOperatorError<1e-10,'Packet control does not match the certified mass operator.');
  }
  assert(Math.abs(sum(newX)-sum(oldX))<1e-10,'Cumulative-mass phase drifted.');
  this.status=this.time>=this.cfg.duration-1e-10?'completed':'running';this.record();return true;
 }
 observe(){
  const n=this.roles.length,x=this.roles.map(a=>a.x),atoms=x.map((v,i)=>this.geo.atom(v,i===n-1?x[0]+1:x[i+1]));
  const service=atoms.map((_,i)=>{const s=Array(4).fill(0);for(let j=0;j<this.cfg.serviceOrder;j++)for(let h=0;h<4;h++)s[h]+=atoms[mod(i-j,n)][h];return s;});
  const centroids=service.map(s=>[s[1]/s[0],s[2]/s[0]]),mass=atoms.map(a=>a[0]),loads=service.map(s=>s[0]);
  return {n,x,mass,loads,centroids,atoms,service,theta:x.map(v=>this.geo.inverse(v))};
 }
 record(){
  const s=this.observe(),n=s.n,mu=1/n,d=gcd(n,this.cfg.serviceOrder),blind=orbitMean(s.mass,d),positions=this.roles.map(a=>a.p.slice());
  let cost=0,lowerBound=0,tracking=0,minPair=Infinity,maxRingLength=0,minMargin=Infinity;
  for(let i=0;i<n;i++){
   const p=positions[i],v=s.service[i],goal=this.geo.feasibleTarget(s.centroids[i]);
   cost+=serviceCost(v,p);lowerBound+=.5*(v[3]-(v[1]**2+v[2]**2)/v[0]);tracking=Math.max(tracking,Math.hypot(p[0]-goal[0],p[1]-goal[1]));
   minMargin=Math.min(minMargin,this.geo.marginAt(p));const next=positions[(i+1)%n];maxRingLength=Math.max(maxRingLength,Math.hypot(p[0]-next[0],p[1]-next[1]));
   for(let j=0;j<i;j++)minPair=Math.min(minPair,Math.hypot(p[0]-positions[j][0],p[1]-positions[j][1]));
  }
  const gaps=s.theta.map((a,i)=>(i===n-1?s.theta[0]+TAU:s.theta[i+1])-a),orders=this.controlOrders();let joint=n;for(const k of orders)joint=gcd(joint,k);
  const row={motionTime:this.time,wallTime:this.wallTime,N:n,serviceK:this.cfg.serviceOrder,serviceGcd:d,jointGcd:joint,
   massError:relError(s.mass,mu),loadError:relError(s.loads,this.cfg.serviceOrder*mu),blindError:relError(blind,mu),targetError:tracking,
   costPerOrder:cost/this.cfg.serviceOrder,centroidCostLowerBound:lowerBound/this.cfg.serviceOrder,centroidExcessCost:(cost-lowerBound)/this.cfg.serviceOrder,minMass:Math.min(...s.mass),minGap:Math.min(...gaps),minRadialMargin:minMargin,
   sampledMinCenterDistance:minPair,minCentroidNorm:Math.min(...s.centroids.map(c=>Math.hypot(...c))),maxRoleNeighborLength:maxRingLength,carrierEdges:this.network.edgeCount||0,
   carrierRevisions:this.network.revisions,packetHops:this.network.total.packetHops,totalByteHops:this.network.total.totalByteHops,
   payloadByteHops:this.network.total.payloadByteHops,communicationRounds:this.network.total.rounds,
   robotInputRms:this.lastEpoch.inputRms,robotInputMax:this.lastEpoch.inputMax,normalizedInputEffort:this.inputEffort,
   boundaryMassRateRms:this.lastEpoch.boundaryRms,boundaryAngularRateRms:this.lastEpoch.boundaryAngleRms,
   motionCostChange:this.lastEpoch.motionCostBefore===null?0:this.lastEpoch.motionCostAfter-this.lastEpoch.motionCostBefore,
   boundaryCostChange:this.lastEpoch.motionCostAfter===null?0:cost/this.cfg.serviceOrder-this.lastEpoch.motionCostAfter,
   translationSpeedRms:Math.sqrt(sum(this.roles.map(a=>a.motion.velocity.reduce((v,x)=>v+x*x,0)))/n)};
  this.minMassEver=Math.min(this.minMassEver,row.minMass);this.minMarginEver=Math.min(this.minMarginEver,minMargin);
  assert(row.minMass>0&&row.minGap>0,'The partition lost a nonempty atom.');this.history.push(row);
  if(this.history.length===1||this.history.length%10===0||this.status==='completed'||this.status==='blocked')this.snapshots.push({motionTime:this.time,wallTime:this.wallTime,ids:this.roles.map(a=>a.id),positions,theta:s.theta,mass:s.mass,centroids:s.centroids});
  return row;
 }
 run(){while(this.step());return this.export();}
 export(){return {config:this.cfg,status:this.status,blockReason:this.blockReason||null,history:this.history,snapshots:this.snapshots,events:this.events,
  final:this.observe(),positions:this.roles.map(a=>a.p.slice()),roleOwners:this.roles.map(a=>a.id),trace:this.network.trace,
  trajectories:Array.from(this.trajectories,([id,points])=>({id,points})),dynamicsMetadata:Motion.metadata(this.cfg),
  inputSeries:Array.from(this.robotInputs,([id,samples])=>({id,samples})),
  finalCarrierEdges:this.network.adj?this.network.adj.flatMap((row,i)=>row.filter(j=>j>i).map(j=>[this.agents[i].id,this.agents[j].id])):[],
  checks:{maxOperatorError:this.maxOperatorError,maxWindowError:this.maxWindowError,maxReconstructionError:this.maxReconstructionError,
   minimumMass:this.minMassEver,minimumRadialMargin:this.minMarginEver,physicalCollisionAvoidanceImplemented:false,
   controllerUsesGlobalMassVector:false,sharedStaticMomentOracle:true,reliableFrozenTopologyDuringEachEpoch:true,
   maxNoSlipResidual:this.maxNoSlipResidual,robotModel:this.cfg.robotModel,latencyClockIsAccountingOnly:true}};}
}
return {Geometry,Network,Simulation,gcd,windowSum,orbitMean,relError,rng};
});
