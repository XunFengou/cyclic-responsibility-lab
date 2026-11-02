import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..'),out=path.join(root,'dist');
fs.mkdirSync(out,{recursive:true});
for(const entry of ['src','assets'])for(const name of fs.readdirSync(path.join(root,entry)))fs.cpSync(path.join(root,entry,name),path.join(out,name),{recursive:true});
const cfg=JSON.parse(fs.readFileSync(path.join(root,'site-config.json'),'utf8'));
const esc=s=>s.replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;');
const inline=s=>esc(s).replace(/`([^`]+)`/g,'<code>$1</code>').replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/\[([^\]]+)\]\((https?:[^)]+)\)/g,'<a href="$2">$1</a>');
function markdown(md){
 let result='',code=false,formula=false,list=false;
 for(const line of md.split(/\r?\n/)){
  if(line.startsWith('```')){if(list){result+='</ul>';list=false;}result+=code?'</code></pre>':'<pre class="formula"><code>';code=!code;continue;}
  if(code){result+=esc(line)+'\n';continue;}
  if(line.trim()==='\\['){result+='<pre class="formula">';formula=true;continue;}if(line.trim()==='\\]'){result+='</pre>';formula=false;continue;}
  if(formula){result+=esc(line)+'\n';continue;}
  if(/^[-*] /.test(line)){if(!list){result+='<ul>';list=true;}result+='<li>'+inline(line.slice(2))+'</li>';continue;}
  if(list){result+='</ul>';list=false;}if(!line.trim())continue;
  const heading=line.match(/^(#{1,3}) (.*)/);result+=heading?`<h${heading[1].length}>${inline(heading[2])}</h${heading[1].length}>`:'<p>'+inline(line)+'</p>';
 }return result+(list?'</ul>':'');
}
const nav='<nav><a class="brand" href="index.html">Cyclic Responsibility Lab</a><a href="Obstacle_Comparison.html">Obstacle comparison</a><a href="Cyclic_Responsibility_Explorer.html">Algebra explorer</a><a href="Experiment_Guide.html">Figure guide</a><a href="theory.html">Models and proofs</a><a href="publication.html">Publication and collaboration</a></nav>';
for(const [file,title,source]of [['theory.html','Robot Dynamics and Coverage Guarantees','Models_and_Guarantees.md'],['publication.html','Publication and Collaboration','Publication_and_Collaboration.md'],['algebra-theory.html','Cyclic Operators and Topology','Theory_and_Topology.md'],['comparison-theory.html','Matched Multi-Obstacle Coverage','Obstacle_Comparison.md']]){
 const body=markdown(fs.readFileSync(path.join(root,'docs',source),'utf8'));
 fs.writeFileSync(path.join(out,file),`<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>${title} | Cyclic Responsibility Lab</title><meta name="description" content="${title}: model equations, precise assumptions, verified experiments, and reproducible research."><link rel="stylesheet" href="style.css"></head><body><main class="document">${nav}${body}<footer><a href="index.html">Return to the simulator</a></footer></main></body></html>`);
}
fs.copyFileSync(path.join(root,'docs/Theory_and_Topology.md'),path.join(out,'Theory_and_Topology.md'));
const guide=path.join(out,'Experiment_Guide.html');let guideText=fs.readFileSync(guide,'utf8').replaceAll('START_HERE.html','index.html').replaceAll('Distributed_Sector_Simulation.html','index.html');
guideText=guideText.replace('<h1>What is each experiment testing?</h1>','<h1>What is each experiment testing?</h1><p><strong>Archived version-3 experiments.</strong> The figures below are the preserved algebra and single-integrator baseline. The main simulator now adds six robot models and separate performance plots. Read <a href="theory.html">the new model guarantees</a> before comparing dynamics.</p>');fs.writeFileSync(guide,guideText);
const additions=[['D01_actual_dynamics_paths','Do the actual plant trajectories differ?','Six native models use the same N=18, service k=5, fixed partition, seed, and initial positions. Initial integrator derivatives are zero. Paths come from integrated plant states.'],['D02_physical_cost_descent','Which cost is actually decreasing?','Recorded J/k is compared with the fixed-partition centroid lower bound. This isolates the cost-descent hypotheses; no monotone envelope is plotted.'],['D03_native_control_inputs','What does each actuator actually command?','The selected robot has velocity, acceleration, jerk, speed/turn, wheel-rate, or generalized-force inputs. Native units appear on separate axes. Euler-Lagrange holding forces need not vanish.'],['D04_algebra_vs_robot_dynamics','Does a new robot model change the gcd operator?','After the matched failure 50 to 49, complementary orders 7 and 8 give identical mass evolution for all six models. Position transients and input requirements differ.']];
let extra='<h2>New multi-dynamics experiments</h2>';for(const [file,title,explanation]of additions)extra+=`<section><h2>${title}</h2><p>${explanation}</p><a href="result/${file}.png"><img loading="lazy" src="result/${file}.png" alt="${title}"></a></section>`;
guideText=guideText.replace('<h2>New distributed experiments</h2>',extra+'<h2>Preserved version-3 distributed experiments</h2>');fs.writeFileSync(guide,guideText);
const routes=['index.html','Cyclic_Responsibility_Explorer.html','Experiment_Guide.html','theory.html','publication.html','algebra-theory.html','Obstacle_Comparison.html','comparison-theory.html'];
for(const route of routes){const p=path.join(out,route);let html=fs.readFileSync(p,'utf8');const url=cfg.origin+'/'+(route==='index.html'?'':route);html=html.replace('</head>',`<link rel="canonical" href="${url}"><meta name="robots" content="index,follow"></head>`);fs.writeFileSync(p,html);}
fs.writeFileSync(path.join(out,'robots.txt'),`User-agent: *\nAllow: /\nSitemap: ${cfg.origin}/sitemap.xml\n`);
fs.writeFileSync(path.join(out,'sitemap.xml'),'<?xml version="1.0" encoding="UTF-8"?><urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">'+routes.map(p=>`<url><loc>${cfg.origin}/${p==='index.html'?'':p}</loc></url>`).join('')+'</urlset>');
// Preserve the old entry filenames as same-Site routes, without a second app.
for(const alias of ['START_HERE.html','Distributed_Sector_Simulation.html'])fs.writeFileSync(path.join(out,alias),'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta http-equiv="refresh" content="0;url=index.html"><link rel="canonical" href="'+cfg.origin+'/"><title>Cyclic Responsibility Lab</title></head><body><a href="index.html">Open the current research simulator</a></body></html>');
for(const file of ['dynamics_checks.json','dynamics_case_summary.csv','obstacle_checks.json'])if(fs.existsSync(path.join(root,'test-results',file)))fs.copyFileSync(path.join(root,'test-results',file),path.join(out,'result',file));
fs.writeFileSync(path.join(out,'_headers'),'/\n  Cache-Control: no-cache\n/index.html\n  Cache-Control: no-cache\n/*.js\n  Cache-Control: no-cache\n');
console.log(JSON.stringify({status:'built',directory:out,routes:routes.length,origin:cfg.origin}));
