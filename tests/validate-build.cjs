'use strict';const fs=require('fs'),path=require('path'),assert=require('assert'),crypto=require('crypto');
const root=path.resolve(__dirname,'..'),dist=path.join(root,'dist');let references=0,pages=0;
for(const name of fs.readdirSync(dist))if(name.endsWith('.html')){
 const content=fs.readFileSync(path.join(dist,name),'utf8');assert(!/[\u3400-\u9fff]/.test(content),name+' has non-English interface text.');pages++;
 for(const m of content.matchAll(/(?:src|href)="([^"]+)"/g)){
  const p=m[1];if(/^(https?:|data:|#|mailto:)/.test(p)||p.includes('&quot;'))continue;
  const file=path.resolve(dist,p.split('#')[0].split('?')[0]);assert(file.startsWith(dist+path.sep));assert(fs.existsSync(file),name+' links to missing '+p);references++;
 }
}
for(const name of ['index.html','style.css','viewer.js','engine.js','dynamics.js','theory.html','publication.html','sitemap.xml','robots.txt','result/dynamics_checks.json','Obstacle_Comparison.html','comparison.js','comparison-viewer.js','comparison-theory.html','result/obstacle_checks.json','result/obstacle_benchmarks.json'])assert(fs.existsSync(path.join(dist,name)),name);
const result={status:'passed',pages,localReferences:references,englishInterface:true,indexHash:crypto.createHash('sha256').update(fs.readFileSync(path.join(dist,'index.html'))).digest('hex'),liveBrowserVerification:false};fs.writeFileSync(path.join(root,'test-results/build_checks.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
