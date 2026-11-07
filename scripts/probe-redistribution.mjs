import fs from 'node:fs';
import load from '../.physics-runtime/mujoco.js';
import T from '../src/tiago-core.js';
import E from '../src/execution-plant.js';
import A from '../src/execution-allocation.js';
import C from '../src/comparison.js';
import L from '../src/engine.js';
import Metrics from '../src/lab-model.js';
import {createRedistributionMission} from '../src/redistribution-mission.mjs';

const M=await load(),xml=fs.readFileSync(new URL('../src/vendor/tiago-pro/planar20.xml',import.meta.url),'utf8');
const Mission=createRedistributionMission({T,E,A,C,L,Metrics});
const options=JSON.parse(process.argv[3]||'{}'),method=process.argv[2]||'repair';
const mission=new Mission(M,xml,method,options);
let previous='';
while(mission.step()){
  const frame=mission.frames.at(-1);
  if(frame.workflow.stage!==previous||Math.abs(frame.time/4-Math.round(frame.time/4))<1e-7){
    previous=frame.workflow.stage;
    console.log(JSON.stringify({time:frame.time,stage:previous,id:frame.workflow.carrierId,object:frame.object.pos,
      pad:frame.object.supportId,force:frame.row.normalForce,slip:frame.row.slipSpeed,jointLimit:frame.row.jointLimitExcess,
      minContact:frame.object.minimumContactDistance,job:mission.jobs[0].progress,rejected:frame.workflow.rejected.slice(-4)}));
  }
  if(options.debug&&['quarantined','complete'].includes(previous))break;
}
const record=mission.export(),folder=new URL('../research/redistribution/',import.meta.url);
fs.mkdirSync(folder,{recursive:true});
fs.writeFileSync(new URL((options.label||'probe')+'.json',folder),JSON.stringify(record));
console.log(JSON.stringify({status:record.status,reason:record.reason,workflow:record.workflow,
  events:record.events.map(({type,time,id,pad,reason})=>({type,time,id,pad,reason}))}));
mission.close();
