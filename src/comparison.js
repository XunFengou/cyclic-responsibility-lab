/* Matched obstacle / order-k comparison. Common quadrature and packet transport.
 * No endpoint clipping. Safety acts on single-integrator velocity commands. */
(function(root,factory){const api=factory(typeof module==='object'&&module.exports?require('./engine.js'):root.SectorLab);if(typeof module==='object'&&module.exports)module.exports=api;else root.CoverageComparison=api;})(typeof globalThis!=='undefined'?globalThis:this,function(Lab){
'use strict';
const TAU=2*Math.PI,mod=(x,n)=>((x%n)+n)%n,sum=a=>a.reduce((s,x)=>s+x,0),dot=(a,b)=>a[0]*b[0]+a[1]*b[1],norm=a=>Math.hypot(...a);
const check=(p,m)=>{if(!p)throw Error(m);},copy=x=>JSON.parse(JSON.stringify(x));
const presets={none:[],three:[{x:.12,y:.1,r:.18},{x:-.43,y:.3,r:.14},{x:.28,y:-.43,r:.16}],five:[{x:0,y:0,r:.18},{x:-.45,y:.34,r:.13},{x:.43,y:.34,r:.14},{x:-.4,y:-.36,r:.15},{x:.43,y:-.35,r:.13}],barrier:[{x:0,y:0,r:.2},{x:0,y:.39,r:.16},{x:0,y:-.39,r:.16}]};
function segmentDistance(a,b,c){const v=[b[0]-a[0],b[1]-a[1]],t=Math.max(0,Math.min(1,dot([c[0]-a[0],c[1]-a[1]],v)/(dot(v,v)||1)));return norm([a[0]+t*v[0]-c[0],a[1]+t*v[1]-c[1]]);}
function cost(moments,points,k){return sum(moments.map((v,i)=>(v[3]-2*(points[i][0]*v[1]+points[i][1]*v[2])+v[0]*dot(points[i],points[i]))/(2*k)));}
class Domain{
 constructor(cfg){
  this.obstacles=copy(cfg.obstacles);this.A=cfg.angularBins;this.R=cfg.radialBins;this.bins=[];this.samples=[];this.cumulative=[[0,0,0,0]];
  for(let a=0;a<this.A;a++){
   const theta=TAU*(a+.5)/this.A,bin={samples:[],moment:[0,0,0,0]};
   for(let r=0;r<this.R;r++){
    const radius=Math.sqrt((r+.5)/this.R),p=[radius*Math.cos(theta),radius*Math.sin(theta)];
    if(this.obstacles.some(o=>norm([p[0]-o.x,p[1]-o.y])<=o.r))continue;
    const density=cfg.density==='uniform'?1:1+3*Math.exp(-((p[0]-.42)**2+(p[1]-.26)**2)/.09)+2*Math.exp(-((p[0]+.38)**2+(p[1]+.4)**2)/.06);
    const s={p,w:density*Math.PI/(this.A*this.R),bin:a,radialIndex:r,theta,radius};bin.samples.push(s);this.samples.push(s);
    const v=[1,p[0],p[1],dot(p,p)];for(let h=0;h<4;h++)bin.moment[h]+=s.w*v[h];
   }
   check(bin.moment[0]>0,'Angular quadrature contains a zero-mass bin; refine the grid or change obstacles.');this.bins.push(bin);
  }
  this.rawMass=sum(this.samples.map(s=>s.w));for(const s of this.samples)s.w/=this.rawMass;
  for(const b of this.bins){b.moment=b.moment.map(v=>v/this.rawMass);this.cumulative.push(b.moment.map((v,h)=>v+this.cumulative.at(-1)[h]));}
  this.cumulative.at(-1)[0]=1;this.total=this.cumulative.at(-1).slice();
 }
 primitive(x){const turns=Math.floor(x),y=x-turns;let lo=0,hi=this.A;while(hi-lo>1){const m=(lo+hi)>>1;if(this.cumulative[m][0]<=y)lo=m;else hi=m;}
  const f=(y-this.cumulative[lo][0])/this.bins[lo].moment[0];return this.total.map((v,h)=>turns*v+this.cumulative[lo][h]+f*this.bins[lo].moment[h]);}
 atom(l,r){const a=this.primitive(l),b=this.primitive(r);return b.map((v,h)=>v-a[h]);}
 inverse(x){const turns=Math.floor(x),y=x-turns;let lo=0,hi=this.A;while(hi-lo>1){const m=(lo+hi)>>1;if(this.cumulative[m][0]<=y)lo=m;else hi=m;}
  return TAU*(turns+(lo+(y-this.cumulative[lo][0])/this.bins[lo].moment[0])/this.A);}
 margin(p,robotRadius=0){return Math.min(1-robotRadius-norm(p),...this.obstacles.map(o=>norm([p[0]-o.x,p[1]-o.y])-o.r-robotRadius));}
 lineOfSight(a,b){return this.obstacles.every(o=>segmentDistance(a,b,[o.x,o.y])>o.r+1e-12);}
}
function sectorAssignment(domain,x,k){const n=x.length,atoms=x.map((v,i)=>domain.atom(v,i===n-1?x[0]+1:x[i+1])),moments=atoms.map((_,i)=>[0,0,0,0].map((_,h)=>sum(Array.from({length:k},(_,j)=>atoms[mod(i-j,n)][h]))));return {moments,atoms,cells:k===n?1:n,owners:null};}
function voronoiAssignment(domain,agents,k){
 const moments=agents.map(()=>[0,0,0,0]),owners=[],cells=new Set();
 for(const s of domain.samples){
  const ranked=agents.map((a,i)=>({i,id:a.id,d:(s.p[0]-a.p[0])**2+(s.p[1]-a.p[1])**2})).sort((a,b)=>a.d-b.d||a.id-b.id),near=ranked.slice(0,k).map(a=>a.i);owners.push(near);
  cells.add(near.map(i=>agents[i].id).sort((a,b)=>a-b).join(','));
  for(const i of near){const v=moments[i];v[0]+=s.w;v[1]+=s.w*s.p[0];v[2]+=s.w*s.p[1];v[3]+=s.w*dot(s.p,s.p);}
 }
 return {moments,owners,cells:cells.size};
}
// Exact 2-D Euclidean projection onto a finite intersection of half planes.
// Candidate optimum is nominal, a face projection, or an intersection of faces.
function projectVelocity(nominal,constraints){
 const feasible=u=>constraints.every(c=>dot(c.a,u)>=c.b-1e-10);let best=[0,0],distance=dot(nominal,nominal);
 const consider=u=>{if(!u.every(Number.isFinite)||!feasible(u))return;const d=(u[0]-nominal[0])**2+(u[1]-nominal[1])**2;if(d<distance){best=u;distance=d;}};consider(nominal);
 for(const c of constraints){const t=(c.b-dot(c.a,nominal))/dot(c.a,c.a);consider([nominal[0]+t*c.a[0],nominal[1]+t*c.a[1]]);}
 for(let i=0;i<constraints.length;i++)for(let j=0;j<i;j++){const a=constraints[i],b=constraints[j],det=a.a[0]*b.a[1]-a.a[1]*b.a[0];if(Math.abs(det)<1e-12)continue;consider([(a.b*b.a[1]-a.a[1]*b.b)/det,(a.a[0]*b.b-a.b*b.a[0])/det]);}
 check(feasible(best),'No feasible safe velocity. Initial conditions must be safe.');return best;
}
function safetyVelocities(agents,centroids,domain,cfg){
 const vlim=cfg.speed/Math.SQRT2,outer=1-cfg.robotRadius-cfg.clearance,pair=2*cfg.robotRadius+cfg.clearance;
 return agents.map((a,i)=>{
  const p=a.p,nominal=centroids[i].map((v,h)=>cfg.gain*(v-p[h])),constraints=[{a:[1,0],b:-vlim},{a:[-1,0],b:-vlim},{a:[0,1],b:-vlim},{a:[0,-1],b:-vlim}];
  if(norm(p)>1e-10)constraints.push({a:p.map(v=>-2*v),b:-cfg.alpha*(outer**2-dot(p,p))});
  for(const o of domain.obstacles){const d=[p[0]-o.x,p[1]-o.y],h=dot(d,d)-(o.r+cfg.robotRadius+cfg.clearance)**2;if(cfg.alpha*h>=2*norm(d)*cfg.speed)continue;constraints.push({a:d.map(v=>2*v),b:-cfg.alpha*h});}
  agents.forEach((b,j)=>{if(j===i)return;const d=[p[0]-b.p[0],p[1]-b.p[1]],h=dot(d,d)-pair**2;if(cfg.alpha*h/2>=2*norm(d)*cfg.speed)return;constraints.push({a:d.map(v=>2*v),b:-cfg.alpha*h/2});});
  return projectVelocity(nominal,constraints);
 });
}
function sweptSafe(agents,next,domain,cfg){
 const outer=1-cfg.robotRadius-cfg.clearance,pair=2*cfg.robotRadius+cfg.clearance;
 for(let i=0;i<agents.length;i++){
  if(norm(next[i])>outer+1e-12)return false;
  for(const o of domain.obstacles)if(segmentDistance(agents[i].p,next[i],[o.x,o.y])<o.r+cfg.robotRadius+cfg.clearance-1e-12)return false;
  for(let j=0;j<i;j++){const a=[agents[i].p[0]-agents[j].p[0],agents[i].p[1]-agents[j].p[1]],b=[next[i][0]-next[j][0],next[i][1]-next[j][1]];if(segmentDistance(a,b,[0,0])<pair-1e-12)return false;}
 }return true;
}
function initialAgents(cfg,domain){
 const random=Lab.rng(cfg.positionSeed??cfg.seed),agents=[],distance=2*cfg.robotRadius+cfg.clearance+.015;
 for(let i=0;i<cfg.N;i++){let p=null;
  for(let tries=0;tries<30000;tries++){
   const onRing=tries===0&&cfg.placement==='ring',theta=onRing?TAU*(i+.5+.1*(random()-.5))/cfg.N:TAU*random(),r=onRing?.74+.06*(random()-.5):Math.sqrt(random())*(1-cfg.robotRadius-cfg.clearance-.01),q=[r*Math.cos(theta),r*Math.sin(theta)];
   if(domain.margin(q,cfg.robotRadius)>=cfg.clearance+.01&&agents.every(a=>norm([a.p[0]-q[0],a.p[1]-q[1]])>=distance)){p=q;break;}
  }check(p,'Could not pack the requested robots safely. Reduce N / robot radius.');agents.push({id:i+1,p});
 }
 // The same physical IDs follow angular order in both methods at initialization.
 agents.sort((a,b)=>mod(Math.atan2(a.p[1],a.p[0]),TAU)-mod(Math.atan2(b.p[1],b.p[0]),TAU));agents.forEach((a,i)=>a.id=i+1);return agents;
}
class Run{
 constructor(method,cfg,domain,initial){this.method=method;this.cfg=cfg;this.domain=domain;this.agents=copy(initial);this.t=0;this.status='ready';this.events=[];this.history=[];this.paths=new Map(this.agents.map(a=>[a.id,[a.p.slice()]]));this.effort=0;this.pathLength=0;this.faultDone=false;this.minimumSweptMargin=Infinity;this.lastInput=0;this.lastPeak=0;this.lastBoundary=0;this.lastMotionDelta=0;this.lastBoundaryDelta=0;this.lastScale=1;this.epoch=0;
  const random=Lab.rng(cfg.seed+7);let m;
  if(cfg.initialMass==='harmonic'){const wave=Array.from({length:cfg.N},(_,i)=>Math.cos(TAU*cfg.harmonicIndex*i/cfg.N)),rms=Math.sqrt(sum(wave.map(v=>v*v))/cfg.N);m=wave.map(v=>1+cfg.harmonicRms*v/rms);}else m=Array.from({length:cfg.N},(_,i)=>cfg.initialMass==='equal'?1:Math.exp(.5*Math.sin(TAU*i/cfg.N)+.4*(random()-.5)));
  let x=cfg.phaseOffset;this.x=m.map(v=>{const old=x;x+=v/sum(m);return old;});this.lastVelocityById=new Map(this.agents.map(a=>[a.id,[0,0]]));this.inputs=[{time:0,byId:this.agents.map(a=>({id:a.id,u:[0,0]}))}];
  this.network=new Lab.Network({carrier:cfg.carrier,radioRange:cfg.radioRange,lineOfSight:cfg.lineOfSight,cutTime:null});this.record();
 }
 assignment(){return this.method==='sector'?sectorAssignment(this.domain,this.x,this.cfg.k):voronoiAssignment(this.domain,this.agents,this.cfg.k);}
 orders(){const k=this.cfg.k,n=this.agents.length;if(this.cfg.policy==='single')return [k];const auxiliary=k<n?k+1:Math.max(1,k-1);if(this.cfg.policy==='switch')return [Math.floor(this.t/this.cfg.switchPeriod)%2?k:auxiliary];return [...new Set([k,auxiliary])];}
 remove(){const index=this.agents.findIndex(a=>a.id===this.cfg.failedId),before=this.agents.length;check(index>=0,'Failed ID is not active.');const beforeState={time:this.t,agents:this.agents.map(a=>({id:a.id,p:a.p.slice()})),x:this.x.slice()};this.agents.splice(index,1);this.x.splice(index,1);this.lastVelocityById.delete(this.cfg.failedId);this.faultDone=true;const afterState={time:this.t,agents:this.agents.map(a=>({id:a.id,p:a.p.slice()})),x:this.x.slice()};this.events.push({type:'announced removal',time:this.t,id:this.cfg.failedId,beforeN:before,afterN:this.agents.length,gcd:Lab.gcd(this.agents.length,this.cfg.k),beforeState,afterState});}
 communicate(){
  const before=this.network.total.rounds;
  if(!this.network.setup(this.agents,this.domain,this.t,this.epoch)){this.status='blocked';this.reason='Disconnected carrier: complete snapshot unavailable.';return false;}
  if(this.network.changed)this.events.push({type:'carrier changed',time:this.t,edges:this.network.edgeCount,revision:this.network.revisions});
  const local=this.agents.map(()=>new Map()),n=this.agents.length,packets=[];
  for(let s=0;s<n;s++)for(let d=0;d<n;d++){const a=this.agents[s],data=[a.id,...a.p,this.x[s]];if(s===d)local[d].set(a.id,data);else packets.push({from:a,to:this.agents[d],epoch:this.epoch,data,receive:v=>local[d].set(v[0],v)});}
  this.network.batch(packets,'matched-snapshot',32);this.lastRounds=this.network.total.rounds-before;
  check(local.every(a=>a.size===n),'Incomplete position/boundary snapshot.');this.snapshot=Array.from(local[0].values()).sort((a,b)=>a[0]-b[0]);
  check(local.every(map=>this.agents.every((a,i)=>{const v=map.get(a.id);return v&&v[1]===a.p[0]&&v[2]===a.p[1]&&v[3]===this.x[i];})),'Packet snapshot disagrees with its epoch state.');
  if(this.lastRounds>this.cfg.roundBudget){this.status='blocked';this.reason='Snapshot exceeds the per-epoch communication-round budget. No motion or boundary update applied.';return false;}
  return true;
 }
 step(){
  if(this.status==='completed'||this.status==='blocked')return false;
  if(this.t>=this.cfg.duration-1e-10){this.status='completed';return false;}
  if(!this.faultDone&&this.cfg.faultTime!==null&&this.t>=this.cfg.faultTime-1e-10){this.remove();this.current=this.assignment();}
  if(!this.communicate()){this.record();return false;}
  const a=this.current,n=this.agents.length,dt=Math.min(this.cfg.dt,this.cfg.duration-this.t),centroids=a.moments.map((v,i)=>v[0]>1e-14?[v[1]/v[0],v[2]/v[0]]:this.agents[i].p.slice()),velocities=safetyVelocities(this.agents,centroids,this.domain,this.cfg),oldCost=cost(a.moments,this.agents.map(a=>a.p),this.cfg.k);
  let scale=1,next;
  for(let j=0;j<45;j++){next=this.agents.map((a,i)=>a.p.map((v,h)=>v+dt*scale*velocities[i][h]));if(sweptSafe(this.agents,next,this.domain,this.cfg)&&cost(a.moments,next,this.cfg.k)<=oldCost+1e-13)break;scale/=2;}
  if(!sweptSafe(this.agents,next,this.domain,this.cfg)||cost(a.moments,next,this.cfg.k)>oldCost+1e-13){scale=0;next=this.agents.map(a=>a.p.slice());}
  this.lastScale=scale;this.lastInput=Math.sqrt(sum(velocities.map(v=>dot(v,v)*scale**2))/n);this.lastPeak=Math.max(...velocities.map(v=>norm(v)*scale));this.effort+=dt*n*this.lastInput**2;
  this.lastVelocityById=new Map(this.agents.map((a,i)=>[a.id,velocities[i].map(v=>v*scale)]));this.inputs.push({time:this.t+dt,byId:this.agents.map(a=>({id:a.id,u:this.lastVelocityById.get(a.id).slice()}))});
  for(let i=0;i<n;i++){const p=this.agents[i].p,q=next[i];this.pathLength+=norm([q[0]-p[0],q[1]-p[1]]);this.minimumSweptMargin=Math.min(this.minimumSweptMargin,1-this.cfg.robotRadius-Math.max(norm(p),norm(q)),...this.domain.obstacles.map(o=>segmentDistance(p,q,[o.x,o.y])-o.r-this.cfg.robotRadius));for(let j=0;j<i;j++)this.minimumSweptMargin=Math.min(this.minimumSweptMargin,segmentDistance([p[0]-this.agents[j].p[0],p[1]-this.agents[j].p[1]],[q[0]-next[j][0],q[1]-next[j][1]],[0,0])-2*this.cfg.robotRadius);}
  this.agents.forEach((a,i)=>{a.p=next[i];this.paths.get(a.id).push(a.p.slice());});
  const afterMotion=this.method==='sector'?a:this.assignment(),afterMotionCost=cost(afterMotion.moments,next,this.cfg.k);this.lastMotionDelta=afterMotionCost-oldCost;this.lastBoundary=0;
  if(this.method==='sector'&&this.cfg.boundary!=='fixed'){
   const orders=this.orders(),loads=orders.map(k=>Lab.windowSum(a.atoms.map(v=>v[0]),k)),beta=dt*this.cfg.boundaryGain;
   const newX=this.x.map((x,i)=>x+beta*sum(orders.map((k,h)=>loads[h][mod(i+k-1,n)]-loads[h][mod(i-1,n)]))/orders.length);
   this.lastBoundary=Math.sqrt(sum(newX.map((x,i)=>((x-this.x[i])/dt)**2))/n);this.x=newX;
  }
  const finalAssignment=this.method==='sector'?this.assignment():afterMotion;this.lastBoundaryDelta=cost(finalAssignment.moments,next,this.cfg.k)-afterMotionCost;this.t+=dt;this.epoch++;this.status=this.t>=this.cfg.duration-1e-10?'completed':'running';this.record(finalAssignment);return true;
 }
 record(assignment){
  const a=assignment||this.assignment(),n=this.agents.length,k=this.cfg.k,loads=a.moments.map(v=>v[0]),points=this.agents.map(a=>a.p),J=cost(a.moments,points,k),mass=this.x.map((v,i)=>(i===n-1?this.x[0]+1:this.x[i+1])-v),d=Lab.gcd(n,k),blind=Lab.orbitMean(mass,d),pair=Math.min(2,...points.flatMap((p,i)=>points.slice(0,i).map(q=>norm([p[0]-q[0],p[1]-q[1]])-2*this.cfg.robotRadius))),margin=Math.min(...points.map(p=>this.domain.margin(p,this.cfg.robotRadius))),centroidResidual=Math.sqrt(sum(a.moments.map((v,i)=>v[0]>1e-14?((v[1]/v[0]-points[i][0])**2+(v[2]/v[0]-points[i][1])**2):0))/n);
  let visible=0;for(const s of this.domain.samples){let count=0;for(const p of points)if(norm([p[0]-s.p[0],p[1]-s.p[1]])<=this.cfg.sensingRange&&this.domain.lineOfSight(p,s.p))count++;if(count>=k)visible+=s.w;}
  const row={time:this.t,N:n,k,gcd:d,cost:J,nearestCost:this.method==='voronoi'?J:cost(voronoiAssignment(this.domain,this.agents,k).moments,points,k),loadError:Lab.relError(loads,k/n),massError:this.method==='sector'?Lab.relError(mass,1/n):null,blindError:this.method==='sector'?Lab.relError(blind,1/n):null,minAtom:this.method==='sector'?Math.min(...mass):null,inputRms:this.lastInput,inputPeak:this.lastPeak,boundaryRms:this.lastBoundary,effort:this.effort,pathLength:this.pathLength,clearance:Math.min(margin,pair),sweptClearance:Number.isFinite(this.minimumSweptMargin)?this.minimumSweptMargin:Math.min(margin,pair),centroidResidual,visibleKFraction:visible,emptyLoads:loads.filter(x=>x<1e-14).length,cells:a.cells,packetHops:this.network.total.packetHops,bytes:this.network.total.totalByteHops,rounds:this.network.total.rounds,stepScale:this.lastScale,motionCostDelta:this.lastMotionDelta,boundaryCostDelta:this.lastBoundaryDelta};
  row.carrierEdges=this.network.edgeCount??0;row.carrierRevision=this.network.revisions;this.current=a;this.history.push(row);return row;
 }
 export(){return {method:this.method,status:this.status,reason:this.reason||null,history:this.history,events:this.events,agents:this.agents,paths:Array.from(this.paths,([id,points])=>({id,points})),inputs:this.inputs,x:this.method==='sector'?this.x:null,finalLoads:this.current.moments.map(v=>v[0]),finalCentroids:this.current.moments.map(v=>v[0]>1e-14?[v[1]/v[0],v[2]/v[0]]:null),packetTrace:this.network.trace};}
}
class Comparison{
 constructor(options={}){
  this.cfg=Object.assign({N:18,k:3,obstacles:presets.three,density:'hotspots',angularBins:240,radialBins:18,seed:2718,placement:'ring',initialMass:'unequal',harmonicIndex:1,harmonicRms:.25,phaseOffset:0,boundary:'moving',policy:'complement',dt:.1,boundaryGain:1,gain:1,speed:.45,alpha:3,robotRadius:.012,clearance:.004,duration:40,faultTime:null,failedId:4,carrier:'ring',radioRange:.9,lineOfSight:true,roundBudget:1000000,sensingRange:2.1,switchPeriod:3},copy(options));
  const c=this.cfg;check(Number.isInteger(c.N)&&c.N>=3&&c.N<=80&&Number.isInteger(c.k)&&c.k>=1&&c.k<=c.N,'Require integer 3 <= N <= 80 and 1 <= k <= N.');
  check(Array.isArray(c.obstacles)&&c.obstacles.length<=20,'Use at most 20 circular obstacles.');
  c.obstacles.forEach(o=>check(o&&['x','y','r'].every(h=>Number.isFinite(o[h]))&&o.r>0&&Math.hypot(o.x,o.y)+o.r<=.90,'Each obstacle needs finite x, y, r > 0 and must lie inside radius 0.90.'));
  for(const key of ['dt','boundaryGain','gain','speed','alpha','robotRadius','clearance','duration','radioRange','roundBudget','sensingRange','switchPeriod'])check(Number.isFinite(c[key])&&c[key]>0,'Invalid positive parameter: '+key);
  check(c.dt<=.2&&c.dt*c.boundaryGain<.5,'Require dt <= 0.2 and dt * boundaryGain < 0.5.');check(c.robotRadius<=.06&&c.clearance<=.03,'Robot radius / clearance too large.');
  check(Number.isInteger(c.angularBins)&&c.angularBins>=60&&c.angularBins<=1440&&Number.isInteger(c.radialBins)&&c.radialBins>=8&&c.radialBins<=80,'Invalid quadrature resolution.');
  for(const [key,values]of Object.entries({density:['uniform','hotspots'],placement:['ring','random'],initialMass:['equal','unequal','harmonic'],boundary:['moving','fixed'],policy:['single','complement','switch'],carrier:['ring','range']}))check(values.includes(c[key]),'Unknown '+key);
  check(Number.isInteger(c.harmonicIndex)&&c.harmonicIndex>=1&&c.harmonicIndex<c.N&&Number.isFinite(c.harmonicRms)&&c.harmonicRms>0&&c.harmonicRms<.5,'Invalid harmonic index or RMS amplitude.');
  check(Number.isInteger(c.seed)&&c.seed>=0&&c.seed<=4294967295,'Seed must be a 32-bit unsigned integer.');
  check(c.positionSeed==null||(Number.isInteger(c.positionSeed)&&c.positionSeed>=0&&c.positionSeed<=4294967295),'Position seed must be a 32-bit unsigned integer.');
  check(Number.isFinite(c.phaseOffset)&&c.phaseOffset>=0&&c.phaseOffset<1,'Initial phase must be in [0,1) mass turns.');
  check(c.faultTime===null||(Number.isFinite(c.faultTime)&&c.faultTime>=0&&c.N>c.k&&Number.isInteger(c.failedId)&&c.failedId>=1&&c.failedId<=c.N),'Invalid announced removal. It must leave N >= k.');
  this.domain=new Domain(c);const agents=initialAgents(c,this.domain);this.initial=copy(agents);this.sector=new Run('sector',c,this.domain,agents);this.voronoi=new Run('voronoi',c,this.domain,agents);
 }
 step(){const a=this.sector.step(),b=this.voronoi.step();return a||b;}
 run(){while(this.step());return this.export();}
 export(){return {schema:'matched-coverage-v1',config:this.cfg,quadrature:{points:this.domain.samples.length,rawMass:this.domain.rawMass,angularBins:this.cfg.angularBins,radialBins:this.cfg.radialBins},initial:this.initial,sector:this.sector.export(),voronoi:this.voronoi.export(),assumptions:['Single integrators; piecewise-constant safe velocities; exact swept disk checks.','Known shared static map; replicated quadrature from complete routed snapshots; equal all-gather protocol.','Euclidean sensing cost through obstacles. LOS k-coverage is a diagnostic, not the optimized objective.','Obstacle-clipped demand may have disconnected radial slices; sectors are intersected with free space.','Safety may create stationary traps. No global optimality or unconditional centroid convergence.']};}
}
return {Comparison,Domain,Run,presets,sectorAssignment,voronoiAssignment,projectVelocity,safetyVelocities,sweptSafe,segmentDistance,cost};
});
