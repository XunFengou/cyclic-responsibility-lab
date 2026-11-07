import fs from 'node:fs';
import zlib from 'node:zlib';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import load from '../.physics-runtime/mujoco.js';
import T from '../src/tiago-core.js';
import E from '../src/execution-plant.js';
import A from '../src/execution-allocation.js';
import C from '../src/comparison.js';
import L from '../src/engine.js';
import Metrics from '../src/lab-model.js';
import Colors from '../src/agent-colors.js';
import {createRedistributionMission} from '../src/redistribution-mission.mjs';
import fixture from './dom-fixture.cjs';

const read=p=>fs.readFileSync(new URL('../'+p,import.meta.url));
const dataset=JSON.parse(zlib.gunzipSync(read('assets/result/redistribution-recordings.json.gz')));
const f=fixture(read('src/Load_Redistribution.html').toString()),M=await load(),callbacks=[];
const context=vm.createContext({...f.context,createRedistributionMission,AgentColors:Colors,TiagoCoverage:T,ExecutionPlant:E,
  ExecutionAllocation:A,CoverageComparison:C,SectorLab:L,CoverageLab:Metrics,actualMujoco:M,performance,
  requestAnimationFrame:fn=>{callbacks.push(fn);return callbacks.length;},
  fetch:async url=>url.endsWith('.xml')?{ok:true,text:async()=>read('src/vendor/tiago-pro/planar20.xml').toString()}:{ok:true,json:async()=>dataset}});
vm.runInContext(read('src/lab-plots.js').toString(),context);
const src=read('src/redistribution-viewer.mjs').toString().replace(/^import[^\n]+\n/,'')
  .replace("import('./vendor/mujoco/mujoco.js')","Promise.resolve({default:async()=>actualMujoco})");
await vm.runInContext('(async()=>{'+src+'})()',context);
const node=id=>f.nodes.get(id);
assert.equal(node('rdResults').children.length,4);
assert.equal(node('rdMetrics').children[0].children[1].textContent,'29.0%');
node('rdAfter').onclick();
assert.equal(node('rdMetrics').children[1].children[1].textContent,'14.5%');
node('rdScenario').value='refuge-b';node('rdScenario').onchange();
assert.equal(node('rdResults').children[2].children[2].textContent,'refuge-b');
node('rdTime').value=100;node('rdTime').oninput();
assert(node('rdEvents').children.some(e=>e.textContent.includes('refuge-b'))||node('rdSupport').textContent.includes('refuge-b'));
assert(f.paintCalls.includes('arc')&&f.paintCalls.includes('fillRect'));
await node('rdRun').onclick();
callbacks.shift()();
assert(node('rdRunStatus').textContent.includes('0.1 s'));
node('rdStop').onclick();
assert.equal(node('rdRunStatus').textContent,'Trial stopped.');
console.log(JSON.stringify({status:'passed',replayCases:dataset.scenarios.length,actualPhysicsStep:true,browserEngine:false}));
