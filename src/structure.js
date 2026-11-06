/* Algebra / protocol experiments. Frozen geometric snapshots are distinguished
 * from physical closed-loop simulations. No asymptotic communication winner is assumed. */
(function(root,factory){const api=factory(typeof module==='object'&&module.exports?require('./engine.js'):root.SectorLab,typeof module==='object'&&module.exports?require('./comparison.js'):root.CoverageComparison);if(typeof module==='object'&&module.exports)module.exports=api;else root.StructureLab=api;})(typeof globalThis!=='undefined'?globalThis:this,function(Lab,C){
'use strict';
const TAU=2*Math.PI,mod=(x,n)=>((x%n)+n)%n,sum=a=>a.reduce((s,x)=>s+x,0),clone=x=>JSON.parse(JSON.stringify(x));
function check(ok,message){if(!ok)throw Error(message);}
function validate(N,k){check(Number.isInteger(N)&&N>=2&&N<=5000&&Number.isInteger(k)&&k>=1&&k<=N,'Require integer 2 <= N <= 5000 and 1 <= k <= N.');}
function windows(m,k){const n=m.length,out=[];let v=0;for(let j=0;j<k;j++)v+=m[mod(-j,n)];out.push(v);for(let i=1;i<n;i++){v+=m[i]-m[mod(i-k,n)];out.push(v);}return out;}
function mode(N,k,orders=[k],beta=.2){
 validate(N,k);check(orders.length>0&&orders.every(h=>Number.isInteger(h)&&h>=1&&h<=N),'Invalid boundary orders.');check(beta>0&&beta<.5,'Require 0 < beta < 1/2.');
 const d=Lab.gcd(N,k),common=orders.reduce((g,h)=>Lab.gcd(g,h),N);let positiveGap=Infinity,contraction=0,slowIndex=1;
 for(let j=1;j<N;j++){const lambda=sum(orders.map(h=>4*Math.sin(Math.PI*j*h/N)**2))/orders.length;if(lambda>1e-12)positiveGap=Math.min(positiveGap,lambda);const factor=Math.abs(1-beta*lambda);if(factor>contraction){contraction=factor;slowIndex=j;}}
 return {N,k,d,q:N/d,s:k/d,blindDimension:d-1,blindFrequencies:Array.from({length:d-1},(_,j)=>(j+1)*N/d),orders,commonBlindDimension:common-1,gap:common===1?positiveGap:0,positiveGap:Number.isFinite(positiveGap)?positiveGap:0,contraction:common===1?contraction:1,slowIndex,stepsFor1e6:common===1?Math.ceil(Math.log(1e-6)/Math.log(contraction)):null,beta};
}
function adjacent(N,k){return k<N?k+1:k-1;}
function shortMode(N,k,orders=[k]){const m=mode(N,k,orders);delete m.blindFrequencies;return m;}
function scan(N,k){validate(N,k);return {fixedN:Array.from({length:Math.min(32,N)},(_,j)=>shortMode(N,j+1)),fixedK:Array.from({length:31},(_,j)=>N+j-15).filter(n=>n>=Math.max(3,k)&&n<=5000).map(n=>shortMode(n,k)),large:[1000,999,998,1001,970].map(n=>({single:shortMode(n,n===970?25:21),complement:shortMode(n,n===970?25:21,[n===970?25:21,n===970?26:22])}))};}
function survivors(N,failedIds){check(Array.isArray(failedIds)&&new Set(failedIds).size===failedIds.length&&failedIds.every(id=>Number.isInteger(id)&&id>=1&&id<=N),'Invalid failed IDs.');const ids=Array.from({length:N},(_,i)=>i+1).filter(id=>!failedIds.includes(id));check(ids.length>=2,'Too few survivors.');const x=ids.map(id=>(id-1)/N),mass=x.map((v,i)=>(i===x.length-1?x[0]+1:x[i+1])-v);return {ids,x,mass};}
function faultClass(N,k,failedIds){const f=survivors(N,failedIds);check(f.ids.length>=k,'Failure leaves fewer than k robots.');const d=Lab.gcd(f.ids.length,k);return {initialN:N,failedIds,...shortMode(f.ids.length,k),massError:Lab.relError(f.mass,1/f.ids.length),blindResidue:Lab.relError(Lab.orbitMean(f.mass,d),1/f.ids.length),mass:f.mass};}
function recovery(N,k,failedIds,policy='single',epochs=1600){
 validate(N,k);const f=survivors(N,failedIds),n=f.ids.length;check(n>=k&&['single','complement','switch'].includes(policy),'Invalid recovery experiment.');let m=f.mass.slice();const rows=[],beta=.2,h=adjacent(n,k);let firstBalanced=null;
 for(let t=0;t<=epochs;t++){const massError=Lab.relError(m,1/n);if(firstBalanced===null&&massError<1e-6)firstBalanced=t;if(t%8===0||t===epochs)rows.push({epoch:t,massError,serviceError:Lab.relError(windows(m,k),k/n),blindError:Lab.relError(Lab.orbitMean(m,Lab.gcd(n,k)),1/n)});if(t===epochs)break;const K=policy==='single'?[k]:policy==='switch'?[Math.floor(t/15)%2?k:h]:[k,h];m=m.map((v,i)=>(1-2*beta)*v+beta*sum(K.map(a=>m[(i+a)%n]+m[mod(i-a,n)]))/K.length);}
 return {policy,N:n,k,rows,finalMass:m,firstBalancedEpoch:firstBalanced,beta,switchWindow:15,classification:shortMode(n,k,policy==='single'?[k]:[k,h])};
}
function sectorOwners(ids,x,u,k){const y=mod(u-x[0],1)+x[0];let i=x.findIndex((a,j)=>y>=a&&y<(j===x.length-1?x[0]+1:x[j+1]));if(i<0)i=x.length-1;return Array.from({length:k},(_,j)=>ids[(i+j)%ids.length]);}
function cyclicChurn(ids,x,removed,k){const keep=ids.map((id,i)=>removed.includes(id)?-1:i).filter(i=>i>=0),newIds=keep.map(i=>ids[i]),newX=keep.map(i=>x[i]),cuts=[0,1,...x.map(v=>mod(v,1)),...newX.map(v=>mod(v,1))].sort((a,b)=>a-b);let turnover=0,lost=0;
 for(let j=1;j<cuts.length;j++){const w=cuts[j]-cuts[j-1];if(w<1e-14)continue;const u=(cuts[j]+cuts[j-1])/2,a=sectorOwners(ids,x,u,k),b=sectorOwners(newIds,newX,u,k);turnover+=w*b.filter(id=>!a.includes(id)).length/k;lost+=w*a.filter(id=>removed.includes(id)).length/k;}return {turnover,lost};
}
function bestEqualPhase(domain,agents,k){const n=agents.length,candidates=new Set([0]),coeff=agents.map((_,i)=>{const values=[0,0,0,.5];for(let h=0;h<k;h++){const p=agents[(i+h)%n].p;values[0]+=(p[0]**2+p[1]**2)/(2*k);values[1]-=p[0]/k;values[2]-=p[1]/k;}return values;});for(const b of domain.cumulative)for(let i=0;i<n;i++)candidates.add(mod(b[0]-i/n,1));let best={cost:Infinity,phase:0};for(const phase of candidates){let cost=0;for(let i=0;i<n;i++){const a=domain.atom(phase+i/n,phase+(i+1)/n);for(let h=0;h<4;h++)cost+=a[h]*coeff[i][h];}if(cost<best.cost)best={cost,phase};}return {...best,candidates:candidates.size,scope:'Offline global phase optimization for the piecewise-linear angular quadrature model; not charged as an online distributed controller.'};}
function symmetricCoincidence(N=18,k=3){const domain=new C.Domain({angularBins:8*N,radialBins:8,obstacles:[],density:'uniform'}),agents=Array.from({length:N},(_,i)=>({id:i+1,p:[.7*Math.cos(TAU*i/N),.7*Math.sin(TAU*i/N)]})),phase=(k-2)/(2*N),x=agents.map((_,i)=>phase+i/N),a=C.sectorAssignment(domain,x,k),b=C.voronoiAssignment(domain,agents,k);let error=0;for(let i=0;i<N;i++)for(let h=0;h<4;h++)error=Math.max(error,Math.abs(a.moments[i][h]-b.moments[i][h]));return {N,k,phase,maxMomentDifference:error,cyclicCost:C.cost(a.moments,agents.map(a=>a.p),k),voronoiCost:C.cost(b.moments,agents.map(a=>a.p),k),ownerSets:b.cells,scope:'Uniform angular demand, regular ring, phase-aligned sectors; the two assignment families coincide away from ties.'};}
function faultLocations(domain,agents,k){
 const N=agents.length;check(k<N,'Location study requires k < N.');const ids=agents.map(a=>a.id),x=agents.map((_,i)=>i/N),oldVor=C.voronoiAssignment(domain,agents,k),oldSector=C.sectorAssignment(domain,x,k),rows=[];
 for(let i=0;i<N;i++){const remaining=agents.filter((_,j)=>j!==i),newX=x.filter((_,j)=>j!==i),v=C.voronoiAssignment(domain,remaining,k),s=C.sectorAssignment(domain,newX,k);let turnover=0;
  for(let j=0;j<domain.samples.length;j++){const old=oldVor.owners[j].map(a=>agents[a].id),now=v.owners[j].map(a=>remaining[a].id);turnover+=domain.samples[j].w*now.filter(id=>!old.includes(id)).length/k;}
  const cyclic=cyclicChurn(ids,x,[agents[i].id],k),f=faultClass(N,k,[i+1]);rows.push({failedId:agents[i].id,sectorPreLoad:oldSector.moments[i][0],voronoiPreLoad:oldVor.moments[i][0],sectorTurnover:cyclic.turnover,voronoiTurnover:turnover,voronoiMinimalTurnover:oldVor.moments[i][0]/k,sectorBlindResidue:f.blindResidue,sectorPostLoadError:Lab.relError(s.moments.map(a=>a[0]),k/(N-1)),voronoiPostLoadError:Lab.relError(v.moments.map(a=>a[0]),k/(N-1)),sectorPostCost:C.cost(s.moments,remaining.map(a=>a.p),k),voronoiPostCost:C.cost(v.moments,remaining.map(a=>a.p),k)});
 }return {N,k,rows,assumption:'Fixed common positions; equal initial cyclic masses; one announced removal; both rules replace one membership only where needed.'};
}
function nestedGeometry(options={}){const cfg=Object.assign({minN:18,maxN:35,k:7,fixedN:30,maxK:12,seed:31415},options),p=new C.Comparison({N:cfg.maxN,k:cfg.k,placement:'random',seed:cfg.seed,angularBins:60,radialBins:8,duration:.2}),random=Lab.rng(cfg.seed+19),pool=clone(p.initial);for(let i=pool.length-1;i>0;i--){const j=Math.floor(random()*(i+1));[pool[i],pool[j]]=[pool[j],pool[i]];}
 const state=n=>pool.slice(0,n).sort((a,b)=>mod(Math.atan2(a.p[1],a.p[0]),TAU)-mod(Math.atan2(b.p[1],b.p[0]),TAU));
 const evaluate=(n,k)=>{const agents=state(n),x=agents.map((_,i)=>i/n),s=C.sectorAssignment(p.domain,x,k),v=C.voronoiAssignment(p.domain,agents,k),phase=bestEqualPhase(p.domain,agents,k);return {...shortMode(n,k),sectorCost:C.cost(s.moments,agents.map(a=>a.p),k),sectorBestPhaseCost:phase.cost,bestPhase:phase.phase,voronoiCost:C.cost(v.moments,agents.map(a=>a.p),k),sectorLoadError:Lab.relError(s.moments.map(a=>a[0]),k/n),voronoiLoadError:Lab.relError(v.moments.map(a=>a[0]),k/n),voronoiCells:v.cells,activeIds:agents.map(a=>a.id)};};
 return {config:cfg,masterPositions:p.initial,selectionOrder:pool.map(a=>a.id),population:Array.from({length:cfg.maxN-cfg.minN+1},(_,i)=>evaluate(cfg.minN+i,cfg.k)),orders:Array.from({length:cfg.maxK},(_,i)=>evaluate(cfg.fixedN,i+1)),scope:'Nested physical site sets; survivors never move; shared demand quadrature; fixed-position assignments, not closed-loop trajectories.'};
}
function dispatch(network,items,type,bytes){const remote=[];for(const p of items)if(p.from.id===p.to.id)p.receive(p.data.slice());else remote.push({...p,epoch:network.epoch});if(remote.length)network.batch(remote,type,bytes);}
function fullSnapshot(network,agents,x,time,type='shared-snapshot'){const received=agents.map(()=>new Map());dispatch(network,agents.flatMap(a=>agents.map((b,i)=>({from:a,to:b,data:[a.id,...a.p,x[agents.indexOf(a)]??time],receive:v=>received[i].set(v[0],v)}))),type,32);return received;}
function localSector(network,domain,agents,x,k,orders){
 const n=agents.length,right=Array(n),atoms=Array(n),service=Array(n),max=Math.max(k,...orders),loads=agents.map(()=>Array(orders.length).fill(0));
 dispatch(network,agents.map((a,i)=>({from:agents[(i+1)%n],to:a,data:[x[(i+1)%n]],receive:v=>{right[i]=v[0]+(i===n-1?1:0);}})),'sector-boundary',8);
 for(let i=0;i<n;i++){atoms[i]=domain.atom(x[i],right[i]);service[i]=atoms[i].slice();orders.forEach((h,j)=>{if(h===1)loads[i][j]=atoms[i][0];});}
 let relay=atoms.map(a=>a.slice()),aggregate=atoms.map(a=>a[0]);
 for(let step=1;step<max;step++){const received=Array(n),width=step<k?32:8;dispatch(network,agents.map((a,i)=>({from:agents[mod(i-1,n)],to:a,data:width===32?relay[mod(i-1,n)]:[relay[mod(i-1,n)][0]],receive:v=>{received[i]=v;}})),'sector-moment',width);relay=received;
  for(let i=0;i<n;i++){aggregate[i]+=received[i][0];if(step<k)for(let h=0;h<4;h++)service[i][h]+=received[i][h];orders.forEach((h,j)=>{if(h===step+1)loads[i][j]=aggregate[i];});}
 }
 const past=Array(n),future=agents.map(()=>Array(orders.length));dispatch(network,agents.map((a,i)=>({from:agents[mod(i-1,n)],to:a,data:loads[mod(i-1,n)],receive:v=>{past[i]=v;}})),'sector-previous-load',8*orders.length);
 for(let j=0;j<orders.length;j++)dispatch(network,agents.map((a,i)=>({from:agents[(i+orders[j]-1)%n],to:a,data:[loads[(i+orders[j]-1)%n][j]],receive:v=>{future[i][j]=v[0];}})),'sector-future-load',8);
 const reference=C.sectorAssignment(domain,x,k);let error=0;for(let i=0;i<n;i++)for(let h=0;h<4;h++)error=Math.max(error,Math.abs(service[i][h]-reference.moments[i][h]));check(error<1e-9,'Routed sector moment mismatch.');
 const newX=x.map((v,i)=>v+.2*sum(future[i].map((a,j)=>a-past[i][j]))/orders.length),oldMass=atoms.map(v=>v[0]);for(let i=0;i<n;i++){const actual=(i===n-1?newX[0]+1:newX[i+1])-newX[i],expected=.6*oldMass[i]+.2*sum(orders.map(h=>oldMass[(i+h)%n]+oldMass[mod(i-h,n)]))/orders.length;check(Math.abs(actual-expected)<1e-9,'Routed boundary operator mismatch.');}
 return {error,moments:service,newX};
}
function cachedVoronoi(network,domain,agents,time,speed,caches){
 const n=agents.length,received=[];let requests=0,certifiedBefore=0;
 // Every receiver knows its own exact position and a speed-bounded ball for each peer.
 for(let i=0;i<n;i++)if(!caches.has(agents[i].id))caches.set(agents[i].id,new Map());
 const missing=agents.some(a=>agents.some(b=>a.id!==b.id&&!caches.get(a.id).has(b.id)));
 if(missing){const data=fullSnapshot(network,agents,[],time,'voronoi-initial-snapshot');for(let i=0;i<n;i++)for(const [id,v]of data[i])caches.get(agents[i].id).set(id,{p:[v[1],v[2]],time});}
 const queries=[];
 function classify(i,s,collect){const own=agents[i],ownD=(s.p[0]-own.p[0])**2+(s.p[1]-own.p[1])**2,di=Math.sqrt(ownD),uncertain=[];let definite=0,possible=0;
  for(let j=0;j<n;j++){if(i===j)continue;const c=caches.get(own.id).get(agents[j].id),b=speed*Math.max(0,time-c.time),dx=s.p[0]-c.p[0],dy=s.p[1]-c.p[1],sq=dx*dx+dy*dy;
   if(b===0){const closer=sq<ownD||(sq===ownD&&agents[j].id<own.id);definite+=Number(closer);possible+=Number(closer);continue;}
   const distance=Math.sqrt(sq),lo=Math.max(0,distance-b),hi=distance+b;if(hi<di){definite++;possible++;}else if(lo<=di){possible++;uncertain.push(j);}
  }
  if(definite>=network.order)return false;if(possible<network.order)return true;if(collect)for(const j of uncertain)collect.add(j);return null;
 }
 for(let i=0;i<n;i++){const needed=new Set();for(const s of domain.samples){const known=classify(i,s,needed);if(known!==null)certifiedBefore++;}for(const j of needed)queries.push({i,j});}
 dispatch(network,queries.map(({i,j})=>({from:agents[i],to:agents[j],data:[agents[i].id,agents[j].id],receive:()=>{received.push({i,j});}})),'voronoi-query',16);requests=queries.length;
 dispatch(network,received.map(({i,j})=>({from:agents[j],to:agents[i],data:[...agents[j].p,time],receive:v=>{caches.get(agents[i].id).set(agents[j].id,{p:[v[0],v[1]],time:v[2]});}})),'voronoi-response',24);
 const moments=agents.map(()=>[0,0,0,0]);for(let i=0;i<n;i++)for(const s of domain.samples){const owns=classify(i,s,null);check(owns!==null,'Uncertified membership after exact responses.');if(owns){const v=moments[i];v[0]+=s.w;v[1]+=s.w*s.p[0];v[2]+=s.w*s.p[1];v[3]+=s.w*(s.p[0]**2+s.p[1]**2);}}
 const reference=C.voronoiAssignment(domain,agents,network.order);let error=0;for(let i=0;i<n;i++)for(let h=0;h<4;h++)error=Math.max(error,Math.abs(moments[i][h]-reference.moments[i][h]));check(error<1e-9,'Certified Voronoi moment mismatch.');
 return {error,moments,requests,certifiedFraction:certifiedBefore/(n*domain.samples.length)};
}
class Replay{
 constructor(domain,frames,options={}){check(frames.length>0,'Empty replay.');this.domain=domain;this.frames=clone(frames);this.cfg=Object.assign({k:3,orders:[3,4],speed:.45,carrier:'ring',radioRange:.9,frozen:false},options);check(['ring','path','radio'].includes(this.cfg.carrier)&&Number.isFinite(this.cfg.speed)&&this.cfg.speed>0,'Invalid replay carrier or speed.');check(frames.every((f,i)=>f.agents.length<=80&&f.x.length===f.agents.length&&this.cfg.k>=1&&Number.isInteger(this.cfg.k)&&this.cfg.k<=f.agents.length&&this.cfg.orders.every(h=>Number.isInteger(h)&&h>=1&&h<=f.agents.length)&&new Set(f.agents.map(a=>a.id)).size===f.agents.length&&Number.isFinite(f.time)&&(i===0||f.time>frames[i-1].time)),'Invalid replay frames or orders.');this.index=0;this.rows=[];this.caches=new Map();this.status='ready';this.networks={};for(const name of ['sector','voronoi','snapshot'])this.networks[name]=new Lab.Network({carrier:this.cfg.carrier==='radio'?'range':'ring',radioRange:this.cfg.radioRange,lineOfSight:true,cutTime:this.cfg.carrier==='path'?0:null});}
 step(){if(this.index>=this.frames.length||this.status==='blocked')return false;const f=this.frames[this.index],agents=f.agents,previous=this.frames[this.index-1];
  if(previous){const old=new Map(previous.agents.map(a=>[a.id,a.p]));for(const a of agents)if(old.has(a.id)){const distance=Math.hypot(a.p[0]-old.get(a.id)[0],a.p[1]-old.get(a.id)[1]);check(distance<=this.cfg.speed*(f.time-previous.time)+1e-9,'Trace violates the advertised speed bound.');if(this.cfg.frozen)check(distance<1e-12,'Known-frozen replay contains moving positions.');}}
  const row={epoch:this.index,time:f.time,N:agents.length,protocols:{}};
  for(const [name,net]of Object.entries(this.networks)){const before={...net.total},copy=clone(agents);if(!net.setup(copy,this.domain,f.time,this.index)){this.status='blocked';this.reason='Common replay carrier is disconnected.';return false;}net.order=this.cfg.k;
   const discovery=net.total.totalByteHops-before.totalByteHops;let result;
   if(name==='sector')result=localSector(net,this.domain,copy,f.x,this.cfg.k,this.cfg.orders);else if(name==='voronoi')result=cachedVoronoi(net,this.domain,copy,f.time,this.cfg.frozen?0:this.cfg.speed,this.caches);else {const data=fullSnapshot(net,copy,f.x,f.time);check(data.every(m=>m.size===copy.length),'Incomplete common snapshot.');result={error:0};}
   const oldIds=previous?.agents.map(a=>a.id)||[],removed=oldIds.filter(id=>!agents.some(a=>a.id===id));if(removed.length)dispatch(net,copy.map(a=>({from:copy[0],to:a,data:removed,receive:()=>{}})),'common-announcement',8*removed.length);
   row.protocols[name]={byteHops:net.total.totalByteHops-before.totalByteHops,payloadByteHops:net.total.payloadByteHops-before.payloadByteHops,packetHops:net.total.packetHops-before.packetHops,rounds:net.total.rounds-before.rounds,discoveryByteHops:discovery,totalByteHops:net.total.totalByteHops,totalRounds:net.total.rounds,error:result.error,queries:result.requests??0,certifiedFraction:result.certifiedFraction??null};
  }this.rows.push(row);this.index++;this.status=this.index===this.frames.length?'completed':'running';return true;
 }
 run(){while(this.step());return this.export();}
 export(){return {status:this.status,reason:this.reason||null,config:this.cfg,rows:this.rows,trace:this.frames,packetTraces:Object.fromEntries(Object.entries(this.networks).map(([k,n])=>[k,n.trace])),scope:'Responsibility-layer replay on identical frozen snapshots; not a robot controller or a full-stack safety communication guarantee.',sharedSafetyReading:'If every epoch already provides the common full snapshot, both assignments are reconstructed locally: zero additional responsibility packets for either method.'};}
}
function makeTrace(options={}){const p=new C.Comparison(Object.assign({N:12,k:3,angularBins:60,radialBins:8,duration:2,dt:.2,initialMass:'unequal'},options)),source=options.traceSource==='voronoi'?p.voronoi:p.sector,frames=[];const capture=()=>frames.push({time:source.t,agents:source.agents.map(a=>({id:a.id,p:a.p.slice()})),x:source.x.slice()});capture();while(source.step())capture();return {domain:p.domain,frames,config:p.cfg};}
class Heap{
 constructor(){this.a=[];}
 push(x){let i=this.a.length;this.a.push(x);while(i){const p=(i-1)>>1;if(this.a[p][0]<=x[0])break;this.a[i]=this.a[p];i=p;}this.a[i]=x;}
 pop(){const a=this.a,first=a[0],last=a.pop();if(a.length){let i=0;while(2*i+1<a.length){let j=2*i+1;if(j+1<a.length&&a[j+1][0]<a[j][0])j++;if(a[j][0]>=last[0])break;a[i]=a[j];i=j;}a[i]=last;}return first;}
}
class CapacitySolver{
 constructor(domain,agents,k){const N=agents.length,Q=domain.samples.length;check(N<=32&&Q<=700&&k<=N,'Capacity reference limit: 32 robots and 700 quadrature points.');this.domain=domain;this.agents=agents;this.k=k;this.source=0;this.qStart=1;this.rStart=Q+1;this.sink=Q+N+1;this.graph=Array.from({length:this.sink+1},()=>[]);this.potential=Array(this.graph.length).fill(0);this.assignmentEdges=[];this.flow=0;this.iterations=0;this.status='running';
  const add=(u,v,cap,cost)=>{const a={to:v,cap,cost,rev:this.graph[v].length,initial:cap},b={to:u,cap:0,cost:-cost,rev:this.graph[u].length,initial:0};this.graph[u].push(a);this.graph[v].push(b);return a;};
  domain.samples.forEach((s,q)=>{add(0,this.qStart+q,k*s.w,0);this.assignmentEdges[q]=agents.map((a,i)=>add(this.qStart+q,this.rStart+i,s.w,((s.p[0]-a.p[0])**2+(s.p[1]-a.p[1])**2)/(2*k)));});agents.forEach((_,i)=>add(this.rStart+i,this.sink,k/N,0));
 }
 step(count=40){for(let round=0;round<count&&this.status==='running';round++){
  if(this.k-this.flow<1e-10){this.status='completed';break;}check(this.iterations++<30000,'Capacity solver iteration limit.');const n=this.graph.length,dist=Array(n).fill(Infinity),parent=Array(n).fill(null),heap=new Heap();dist[0]=0;heap.push([0,0]);
  while(heap.a.length){const [d,u]=heap.pop();if(d!==dist[u])continue;for(let j=0;j<this.graph[u].length;j++){const e=this.graph[u][j];if(e.cap<1e-13)continue;const reduced=e.cost+this.potential[u]-this.potential[e.to];check(reduced>=-1e-8,'Negative reduced cost in min-cost flow.');const next=d+Math.max(0,reduced);if(next<dist[e.to]-1e-13){dist[e.to]=next;parent[e.to]=[u,j];heap.push([next,e.to]);}}}
  check(Number.isFinite(dist[this.sink]),'Capacity assignment infeasible.');for(let i=0;i<n;i++)if(Number.isFinite(dist[i]))this.potential[i]+=dist[i];let amount=this.k-this.flow;
  for(let v=this.sink;v!==0;){const [u,j]=parent[v];amount=Math.min(amount,this.graph[u][j].cap);v=u;}
  for(let v=this.sink;v!==0;){const [u,j]=parent[v],e=this.graph[u][j];e.cap-=amount;this.graph[v][e.rev].cap+=amount;v=u;}this.flow+=amount;
 }return this.status==='running';}
 result(){check(this.status==='completed','Capacity solve not complete.');const loads=this.agents.map(()=>0);let cost=0,fractionalPoints=0;
  this.assignmentEdges.forEach(edges=>{let positive=0;edges.forEach((e,i)=>{const f=e.initial-e.cap;loads[i]+=f;cost+=f*e.cost;if(f>1e-10)positive++;});if(positive>this.k)fractionalPoints++;});
  // A feasible residual potential certifies absence of negative-cost cycles.
  const d=Array(this.graph.length).fill(0);let changed=false;for(let t=0;t<d.length;t++){changed=false;for(let u=0;u<d.length;u++)for(const e of this.graph[u])if(e.cap>1e-11&&d[e.to]>d[u]+e.cost+1e-10){d[e.to]=d[u]+e.cost;changed=true;}if(!changed)break;check(t<d.length-1,'Negative residual cycle.');}
  const positions=this.agents.map(a=>a.p),x=this.agents.map((_,i)=>i/this.agents.length),cyclic=C.sectorAssignment(this.domain,x,this.k),vor=C.voronoiAssignment(this.domain,this.agents,this.k);
  const phase=bestEqualPhase(this.domain,this.agents,this.k);return {cost,loads,loadError:Lab.relError(loads,this.k/loads.length),flow:this.flow,iterations:this.iterations,fractionalPoints,residualOptimalityCertificate:!changed,cyclicCost:phase.cost,initialCyclicCost:C.cost(cyclic.moments,positions,this.k),bestCyclicPhase:phase.phase,phaseCandidates:phase.candidates,cyclicLoadError:Lab.relError(cyclic.moments.map(a=>a[0]),this.k/loads.length),voronoiCost:C.cost(vor.moments,positions,this.k),voronoiLoadError:Lab.relError(vor.moments.map(a=>a[0]),this.k/loads.length),N:loads.length,k:this.k,points:this.domain.samples.length,scope:'Centralized fractional capacity-constrained reference on the same quadrature, not a distributed Voronoi controller or a physical trajectory.'};
 }
 run(){while(this.step(100));return this.result();}
}
return {mode,shortMode,scan,adjacent,survivors,faultClass,recovery,faultLocations,cyclicChurn,sectorOwners,bestEqualPhase,symmetricCoincidence,nestedGeometry,localSector,cachedVoronoi,Replay,makeTrace,CapacitySolver};
});
