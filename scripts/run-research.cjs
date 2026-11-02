'use strict';
const fs=require('fs'),path=require('path'),{Simulation}=require('../src/engine');
const root=path.resolve(__dirname,'..'),out=path.join(root,'research');fs.mkdirSync(out,{recursive:true});const rows=[];
function record(name,cfg){const s=new Simulation(cfg);s.record();const d=s.run();d.name=name;fs.writeFileSync(path.join(out,name+'.json'),JSON.stringify(d));const h=s.history.at(-1);rows.push({name,robotModel:s.cfg.robotModel,order:s.cfg.integratorOrder,status:s.status,...h});console.log(name,'tracking',h.targetError,'mass',h.massError,'cost',h.costPerOrder);}
const base={N:18,serviceOrder:5,geometry:'disk',faultTime:null,freezeBoundaries:true,placement:'scattered',duration:40,seed:2718};
for(const robotModel of ['single','double','high','unicycle','differential','lagrangian'])record('fixed_'+robotModel,{...base,robotModel});
for(const r of [4,5])record('fixed_order_'+r,{...base,robotModel:'high',integratorOrder:r});
for(const robotModel of ['single','double','high','unicycle','differential','lagrangian'])record('fault_'+robotModel,{N:50,serviceOrder:7,geometry:'disk',policy:'complement',robotModel,duration:200});
record('annulus_single',{N:50,serviceOrder:7,geometry:'annulus',policy:'complement',duration:200});
const keys=Object.keys(rows[0]),csv=keys.join(',')+'\n'+rows.map(r=>keys.map(k=>JSON.stringify(r[k]??'')).join(',')).join('\n');fs.writeFileSync(path.join(out,'case_summary.csv'),csv);fs.mkdirSync(path.join(root,'assets','result'),{recursive:true});fs.writeFileSync(path.join(root,'assets','result','research_case_summary.csv'),csv);
fs.writeFileSync(path.join(out,'status.json'),JSON.stringify({status:'completed',recordedCases:rows.length,seed:2718,created:new Date().toISOString()},null,2));
