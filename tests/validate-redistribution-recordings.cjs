'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const zlib=require('node:zlib');
const {createHash}=require('node:crypto');
const {gcd}=require('../src/engine.js');

const root=path.resolve(__dirname,'..');
const file=name=>path.join(root,name);
const data=JSON.parse(zlib.gunzipSync(fs.readFileSync(file('assets/result/redistribution-recordings.json.gz'))));
const source=createHash('sha256');
for(const name of ['acquisition-plant.mjs','autonomous-mission.mjs','friction-plant.mjs','redistribution-field.mjs','redistribution-mission.mjs','vendor/tiago-pro/planar20.xml'])source.update(fs.readFileSync(file('src/'+name)));
assert.equal(data.sourceHash,source.digest('hex'));
assert.equal(data.scenarios.length,6);
const close=(a,b)=>Math.abs(a-b)<1e-9;
const study=(id,method='repair')=>data.scenarios.find(s=>s.id===id).trials.find(t=>t.method===method);
let placed=0,confirmed=0,takeovers=0;
for(const scenario of data.scenarios){
  assert.equal(scenario.trials.length,4);
  const initial=scenario.trials[0];
  for(const trial of scenario.trials){
    assert.equal(trial.initialStateHash,initial.initialStateHash,'Unmatched initial poses');
    assert.deepEqual(trial.config,initial.config);
    assert.deepEqual(trial.pads,initial.pads);
    assert.equal(trial.equalityConstraints,0);
    assert.equal(trial.status,'completed');
    assert(trial.pads.every((p,i)=>!i||Math.abs(p.pos[1]-trial.pads[i-1].pos[1])>p.halfSize[1]+trial.pads[i-1].halfSize[1]));
    for(const f of trial.frames){
      const r=f.row;
      assert.equal(r.gcd,gcd(r.N,r.k));
      assert(close(r.fixedLoads.reduce((a,b)=>a+b,0),r.k));
      assert(r.widths.every(a=>a>0)&&close(r.widths.reduce((a,b)=>a+b,0),2*Math.PI));
      assert(f.object.pos.every(Number.isFinite));
      assert(r.normalForce>=0&&r.slipSpeed>=0);
      if(r.transferredLoad>0)assert(f.workflow.placementCertificate);
    }
    for(const s of trial.workflow.safeSetdowns){
      const next=trial.workflow.history.find(x=>x.time>s.time);
      if(next){assert.notEqual(next.id,s.id);takeovers++;}
      const f=trial.frames.find(f=>close(f.time,s.time));
      assert.equal(f.object.supportId,s.padId);
      assert(f.object.stable&&f.object.contact.normalForce<.2);
    }
    if(trial.workflow.placementCertificate){
      placed++;
      const cert=trial.workflow.placementCertificate,f=trial.frames.find(f=>close(f.time,cert.time));
      assert.equal(f.object.supportId,cert.padId);
      assert(f.object.stable&&close(f.row.transferredLoad,trial.config.parcelShare));
    }
    if(trial.workflow.success){
      confirmed++;
      assert(trial.workflow.confirmationCertificate.time>=trial.workflow.placementCertificate.time);
      assert.equal(new Set(trial.workflow.confirmationCertificate.owners).size,trial.config.k);
    }
  }
}
const light=study('light'),first=light.frames[0],last=light.frames.at(-1);
assert.equal(light.workflow.history[0].id,4);
assert(light.workflow.success&&last.row.jointLimitExcess<.004);
assert(light.frames.some(f=>f.object.pos[2]>1.06&&f.object.contact.bilateral&&f.object.supportId===null));
assert(close(first.row.sourceLoad,.29)&&close(last.row.sourceLoad,.145));
assert(last.row.imbalance<first.row.imbalance&&last.row.bestEqualMassMinAngle>first.row.bestEqualMassMinAngle);
for(const id of ['refuge-a','refuge-b']){
  const t=study(id);
  assert.equal(t.workflow.safeSetdowns[0]?.padId,id);
  assert(t.workflow.success&&t.graspHistory.length>=2);
  assert(t.frames.at(-1).row.jointLimitExcess<.004);
}
const failure=study('force-loss');
assert(failure.events.some(e=>e.type==='jaw-force fault'));
assert(failure.events.some(e=>e.type==='grip loss detected'));
assert(!failure.workflow.placementCertificate&&failure.frames.at(-1).row.transferredLoad===0);
const movies=['normal','handover'].map(name=>{
  const stem='assets/result/redistribution-'+name;
  const provenance=JSON.parse(fs.readFileSync(file(stem+'.video.json')));
  const bytes=fs.readFileSync(file(stem+'.mp4'));
  const sha=createHash('sha256').update(bytes).digest('hex');
  assert.equal(sha,provenance.outputSha256??provenance.videoSha256);
  assert(bytes.includes(Buffer.from('ftyp'))&&fs.statSync(file(stem+'-poster.png')).size>1000);
  return {name,sha,bytes:bytes.length};
});
const report={status:'passed',trials:24,placed,confirmed,takeovers,movies,
  initialRms:first.row.imbalance,finalRms:last.row.imbalance,initialBestAngle:first.row.bestEqualMassMinAngle,finalBestAngle:last.row.bestEqualMassMinAngle};
fs.mkdirSync(file('test-results'),{recursive:true});
fs.writeFileSync(file('test-results/redistribution_checks.json'),JSON.stringify(report,null,2));
console.log(JSON.stringify(report));
