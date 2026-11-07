import fs from 'node:fs';
import zlib from 'node:zlib';
import {createHash} from 'node:crypto';
import load from '../.physics-runtime/mujoco.js';
import T from '../src/tiago-core.js';
import E from '../src/execution-plant.js';
import A from '../src/execution-allocation.js';
import C from '../src/comparison.js';
import L from '../src/engine.js';
import Metrics from '../src/lab-model.js';
import {createRedistributionMission} from '../src/redistribution-mission.mjs';

const methods=['cyclic','voronoi','repair','geometricRepair'];
const scenarios=[
  {id:'light',title:'Light parcel · N6, k2',cfg:{duration:100}},
  {id:'refuge-a',title:'Degraded chassis at 35% · N6, k2',cfg:{duration:198,faultMode:'recoverable'}},
  {id:'refuge-b',title:'Degraded chassis at 65% · N6, k2',cfg:{duration:198,faultMode:'recoverable',faultFraction:.65}},
  {id:'force-loss',title:'Reduced jaw force · N6, k2',cfg:{duration:92,faultMode:'structural'}},
  {id:'coprime',title:'Coprime roles · N5, k2',cfg:{N:5,duration:105}},
  {id:'k3',title:'Three independent views · N6, k3',cfg:{k:3,duration:105}},
];
const root=new URL('../',import.meta.url),folder=new URL('research/redistribution/',root);
fs.mkdirSync(folder,{recursive:true});
const model=new URL('src/vendor/tiago-pro/planar20.xml',root);
const signature=createHash('sha256');
for(const file of ['acquisition-plant.mjs','autonomous-mission.mjs','friction-plant.mjs','redistribution-field.mjs','redistribution-mission.mjs'])signature.update(fs.readFileSync(new URL('src/'+file,root)));
signature.update(fs.readFileSync(model));
const sourceHash=signature.digest('hex');
const trialPath=(s,m)=>new URL(`compact-${s.id}-${m}.json`,folder);

function reduced(record){
  const times=new Set(record.events.map(e=>e.time));
  const selected=record.frames.filter((f,i)=>i===0||i===record.frames.length-1||i%5===0||times.has(f.time)||f.workflow.stage!==record.frames[i-1]?.workflow.stage);
  const frames=selected.map(f=>({time:f.time,agents:f.agents,robots:f.robots.map(({id,p,available,active})=>({id,p,available,active})),
    workflow:{stage:f.workflow.stage,carrierId:f.workflow.carrierId,padId:f.workflow.padId,history:f.workflow.history,
      safeSetdowns:f.workflow.safeSetdowns,placementCertificate:f.workflow.placementCertificate,confirmationCertificate:f.workflow.confirmationCertificate,success:f.workflow.success},
    object:{pos:f.object.pos,forceTarget:f.object.forceTarget,supportId:f.object.supportId,stable:f.object.stable,
      contact:{supportForce:f.object.contact.supportForce,normalForce:f.object.contact.normalForce,bilateral:f.object.contact.bilateral},
      maximumJointViolation:f.object.maximumJointViolation,minimumContactDistance:f.object.minimumContactDistance,peakSlip:f.object.peakSlip},
    row:f.row}));
  return {method:record.method,sourceHash,config:record.config,status:record.status,reason:record.reason,equalityConstraints:record.equalityConstraints,
    initialStateHash:createHash('sha256').update(JSON.stringify(record.frames[0].qpos)).digest('hex'),
    obstacles:record.domain.obstacles,pads:record.pads,hotspot:record.hotspot,workflow:record.workflow,graspHistory:record.graspHistory,
    events:record.events.map(({type,time,id,pad,reason,candidates})=>({type,time,id,pad,reason,candidates})),frames};
}

if(process.argv[2]==='--aggregate'){
  const rowKeys=['N','k','gcd','physicalN','available','imbalance','fixedLoads','angles','widths','distortion','minimumAngle',
    'bestEqualMassMinAngle','bestEqualMassRatio','nearestFixedRobotImbalance','transferredLoad','sourceLoad','normalForce',
    'slipSpeed','jointLimitExcess','bytes','backgroundComplete','H','massError','loadError'];
  const dataset={schema:'paired-friction-load-redistribution-v1',sourceHash,scenarios:scenarios.map(s=>({...s,trials:methods.map(m=>{
    const trial=JSON.parse(fs.readFileSync(trialPath(s,m),'utf8'));
    if(trial.sourceHash!==sourceHash)throw Error(`Stale result ${s.id}/${m}`);
    for(const f of trial.frames)f.row=Object.fromEntries(rowKeys.map(k=>[k,f.row[k]]));
    return trial;
  })}))};
  const output=new URL('assets/result/redistribution-recordings.json.gz',root);
  fs.writeFileSync(output,zlib.gzipSync(JSON.stringify(dataset),{level:9}));
  console.log(JSON.stringify({scenarios:scenarios.length,trials:scenarios.length*methods.length,sourceHash}));
}else{
  const scenario=process.argv[2],method=process.argv[3];
  const filter=scenarios.filter(s=>!scenario||scenario==='all'||s.id===scenario);
  if(!filter.length||method&&!methods.includes(method))throw Error('Unknown benchmark scenario or method.');
  const M=await load(),xml=fs.readFileSync(model,'utf8'),Mission=createRedistributionMission({T,E,A,C,L,Metrics});
  for(const s of filter)for(const m of method?[method]:methods){
    const path=trialPath(s,m);
    if(fs.existsSync(path)&&JSON.parse(fs.readFileSync(path,'utf8')).sourceHash===sourceHash)continue;
    const mission=new Mission(M,xml,m,s.cfg);
    while(mission.step()){}
    const record=mission.export(),compact=reduced(record);
    fs.writeFileSync(path,JSON.stringify(compact));
    if(m==='repair'&&['light','refuge-a','refuge-b','force-loss'].includes(s.id))fs.writeFileSync(new URL(`full-${s.id}.json`,folder),JSON.stringify(record));
    const last=compact.frames.at(-1);
    console.log(JSON.stringify({scenario:s.id,method:m,status:record.status,reason:record.reason,stage:record.workflow.stage,
      placed:record.workflow.placementCertificate?.time,confirmed:record.workflow.confirmationCertificate?.time,
      carriers:record.workflow.history.map(x=>x.id),refuges:record.workflow.safeSetdowns.map(x=>x.padId),
      jointLimit:last.row.jointLimitExcess,slip:last.object.peakSlip,finalImbalance:last.row.imbalance}));
    mission.close();
  }
}
