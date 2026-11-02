'use strict';
const assert=require('assert'),fs=require('fs'),path=require('path'),C=require('../src/comparison'),Lab=require('../src/engine');
const groups=[],tests=(name,f)=>{f();groups.push(name);console.log('PASS',name);},sum=a=>a.reduce((s,x)=>s+x,0),near=(a,b,t=1e-10)=>assert(Math.abs(a-b)<t,`${a} != ${b}`);
const base={N:12,k:3,angularBins:120,radialBins:12,duration:4,dt:.2};
tests('Matched states, normalized demand, positive radial marginal, and exact sector moments',()=>{
 const p=new C.Comparison(base);assert.deepStrictEqual(p.sector.agents,p.voronoi.agents);near(sum(p.domain.samples.map(s=>s.w)),1);assert(p.domain.bins.every(b=>b.moment[0]>0));
 assert.notDeepStrictEqual(p.initial,new C.Comparison({...base,seed:31415}).initial);const random=new C.Comparison({...base,placement:'random'});assert.deepStrictEqual(random.sector.agents,random.voronoi.agents);
 const narrow=new C.Comparison({...base,N:3,k:1,obstacles:[{x:0,y:0,r:.9}]});assert(narrow.initial.every(a=>narrow.domain.margin(a.p,narrow.cfg.robotRadius)>narrow.cfg.clearance));
 for(const x of [-1.23,0,.17,.999,2.13]){const v=p.domain.atom(x,x+1);for(let h=0;h<4;h++)near(v[h],p.domain.total[h]);}
 for(const k of [1,3,12]){const a=C.sectorAssignment(p.domain,p.sector.x,k);near(sum(a.moments.map(v=>v[0])),k);assert(a.atoms.every(v=>v[0]>0));if(k===12)assert(a.cells===1);}
});
tests('Actual nearest-k subset selection, dominant moments, ties, and fixed-position inequality',()=>{
 const p=new C.Comparison(base),a=p.voronoi.current;
 a.owners.forEach((owners,j)=>{assert(owners.length===base.k&&new Set(owners).size===base.k);const q=p.domain.samples[j].p,ds=p.voronoi.agents.map(b=>(q[0]-b.p[0])**2+(q[1]-b.p[1])**2);for(const i of owners)for(let h=0;h<ds.length;h++)if(!owners.includes(h))assert(ds[i]<=ds[h]+1e-12);});
 near(sum(a.moments.map(v=>v[0])),base.k);const points=p.initial.map(a=>a.p);assert(C.cost(a.moments,points,base.k)<=C.cost(p.sector.current.moments,points,base.k)+1e-12);
 const ties=C.voronoiAssignment({samples:[{p:[0,0],w:1}]},[{id:9,p:[1,0]},{id:2,p:[-1,0]},{id:3,p:[0,2]}],1);assert.deepStrictEqual(ties.owners,[[1]]);
 const dominant=a.moments[0],brute=p.domain.samples.reduce((v,s,j)=>a.owners[j].includes(0)?v.map((x,h)=>x+s.w*[1,...s.p,s.p[0]**2+s.p[1]**2][h]):v,[0,0,0,0]);dominant.forEach((v,h)=>near(v,brute[h]));
});
tests('Half-plane projection and exact swept tests catch tunneling and robot crossings',()=>{
 assert.deepStrictEqual(C.projectVelocity([2,-1],[{a:[1,0],b:0},{a:[-1,0],b:-1},{a:[0,1],b:0}]),[1,0]);
 const p=new C.Comparison(base),cfg=p.cfg,d=new C.Domain({...cfg,obstacles:[{x:0,y:0,r:.2}]});
 assert(!C.sweptSafe([{p:[-.5,0]}],[[.5,0]],d,cfg));assert(!C.sweptSafe([{p:[-.5,.6]},{p:[.5,.6]}],[[.5,.6],[-.5,.6]],d,cfg));
 assert(C.sweptSafe([{p:[-.5,.6]}],[[-.4,.6]],d,cfg));
});
const cases=[];
tests('Multiple obstacle unions: safe swept paths, finite states, speed bound, and per-motion descent',()=>{
 for(const obstacles of [C.presets.none,C.presets.three,C.presets.five,[{x:.1,y:0,r:.23},{x:-.1,y:0,r:.23}]]){
  const p=new C.Comparison({...base,obstacles});p.run();cases.push(p);
  for(const run of [p.sector,p.voronoi]){assert(run.status==='completed');assert(run.history.every(r=>r.cost>=0&&r.sweptClearance>=p.cfg.clearance-1e-9&&r.inputPeak<=p.cfg.speed+1e-10&&r.motionCostDelta<=2e-12));
   for(const h of run.history)assert(h.nearestCost<=h.cost+1e-11);assert(run.history.every(h=>Math.abs(h.visibleKFraction)<=1+1e-10));}
  assert(p.sector.history.every(h=>h.minAtom>0));
 }
});
tests('Frozen sectors and dynamic Voronoi have nonincreasing assigned cost',()=>{
 const p=new C.Comparison({...base,boundary:'fixed',duration:10});p.run();for(const r of [p.sector,p.voronoi])for(let i=1;i<r.history.length;i++)assert(r.history[i].cost<=r.history[i-1].cost+1e-12);
});
tests('Boundary packets realize the exact positive circulant mass update',()=>{
 for(const policy of ['single','complement','switch']){const p=new C.Comparison({...base,policy}),s=p.sector,m=s.history[0],old=s.x.map((x,i)=>(i===s.x.length-1?s.x[0]+1:s.x[i+1])-x),orders=s.orders(),beta=p.cfg.dt*p.cfg.boundaryGain;s.step();const actual=s.x.map((x,i)=>(i===s.x.length-1?s.x[0]+1:s.x[i+1])-x);for(let i=0;i<old.length;i++)near(actual[i],(1-2*beta)*old[i]+beta*sum(orders.map(k=>old[(i+k)%old.length]+old[(i-k+old.length)%old.length]))/orders.length);near(sum(actual),1);assert(s.history.at(-1).minAtom>0);assert(m.massError>=s.history.at(-1).massError-1e-10);}
});
tests('Announced removal preserves survivor positions and merges adjacent positive masses',()=>{
 for(const failedId of [1,4,12]){const p=new C.Comparison({...base,faultTime:0,failedId,initialMass:'equal'});for(const s of [p.sector,p.voronoi]){const before=new Map(s.agents.map(a=>[a.id,a.p.slice()]));s.remove();s.agents.forEach(a=>assert.deepStrictEqual(a.p,before.get(a.id)));const a=s.assignment();near(sum(a.moments.map(v=>v[0])),3);if(s.method==='sector'){const m=a.atoms.map(v=>v[0]);near(Math.max(...m),2/12);near(Math.min(...m),1/12);}}}
});
tests('50 to 49, k=7: exact six-dimensional residue and complementary/switching repair',()=>{
 const N=49,k=7,beta=.2,initial=Array(N).fill(1/50);initial[12]=2/50;const expected=Math.sqrt(6)/50;near(Lab.relError(Lab.orbitMean(initial,7),1/N),expected);
 const iterate=policy=>{let m=initial.slice();for(let t=0;t<16000;t++){const K=policy==='single'?[7]:policy==='switch'?[Math.floor(t/15)%2?7:8]:[7,8];m=m.map((v,i)=>(1-2*beta)*v+beta*sum(K.map(h=>m[(i+h)%N]+m[(i-h+N)%N]))/K.length);}return m;};
 const single=iterate('single');near(Lab.relError(single,1/N),expected,1e-9);assert(Lab.relError(Lab.windowSum(single,k),k/N)<1e-10);assert(Lab.relError(iterate('complement'),1/N)<1e-9);assert(Lab.relError(iterate('switch'),1/N)<1e-9);
 const p=new C.Comparison({...base,N:50,k:7,duration:.4,faultTime:0,failedId:13,initialMass:'equal',policy:'single'});p.run();near(p.sector.history.at(-1).blindError,expected);assert(p.sector.agents.length===49&&p.voronoi.agents.length===49);
});
tests('Disconnected radio and insufficient full-epoch budgets block both state updates',()=>{
 for(const cfg of [{carrier:'range',radioRange:.001},{roundBudget:1}]){const p=new C.Comparison({...base,...cfg}),x=p.sector.x.slice(),before=JSON.stringify(p.initial);p.step();for(const r of [p.sector,p.voronoi]){assert(r.status==='blocked'&&r.t===0);assert(JSON.stringify(r.agents.map(a=>({id:a.id,p:a.p})))===before);}assert.deepStrictEqual(p.sector.x,x);}
 const p=new C.Comparison({...base,duration:.2});p.run();assert(p.sector.network.total.packetHops===p.voronoi.network.total.packetHops);assert(p.sector.network.trace.some(t=>t.type==='matched-snapshot'));assert(p.sector.network.trace.every(t=>Number.isInteger(t.from)&&Number.isInteger(t.to)));
});
tests('Obstacle LOS is exact and safety does not imply centroid convergence',()=>{
 const p=new C.Comparison({...base,obstacles:[{x:0,y:0,r:.2}]}),r=.2+p.cfg.robotRadius+p.cfg.clearance;assert(!p.domain.lineOfSight([-.5,0],[.5,0]));assert(p.domain.lineOfSight([-.5,.5],[.5,.5]));const u=C.safetyVelocities([{id:1,p:[r,0]}],[[0,0]],p.domain,p.cfg);assert(Math.hypot(...u[0])<1e-10);assert(r>.2);
});
tests('Large arithmetic cases and invalid problem definitions are explicit',()=>{
 assert.deepStrictEqual([1000,999,998,1001].map(n=>Lab.gcd(n,21)),[1,3,1,7]);
 for(const cfg of [{N:2},{k:13},{obstacles:[{x:.9,y:0,r:.2}]},{obstacles:[{x:0,y:0,r:-1}]},{dt:.8},{faultTime:0,N:3,k:3},{density:'invisible'}])assert.throws(()=>new C.Comparison({...base,...cfg}));
});
const report={status:'passed',groups,matchedCases:cases.length,minimumSweptClearance:Math.min(...cases.flatMap(p=>[p.sector,p.voronoi].flatMap(r=>r.history.map(h=>h.sweptClearance)))),maxMotionCostIncrease:Math.max(...cases.flatMap(p=>[p.sector,p.voronoi].flatMap(r=>r.history.map(h=>h.motionCostDelta)))),faultBlindResidue:Math.sqrt(6)/50,liveBrowserVerification:false,scope:'Numerical core tests; browser and static checks reported separately.'};
fs.mkdirSync(path.join(__dirname,'../test-results'),{recursive:true});fs.writeFileSync(path.join(__dirname,'../test-results/obstacle_checks.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report));
