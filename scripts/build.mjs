import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..'),out=path.join(root,'dist');
fs.mkdirSync(out,{recursive:true});
for(const entry of ['src','assets'])for(const name of fs.readdirSync(path.join(root,entry)))fs.cpSync(path.join(root,entry,name),path.join(out,name),{recursive:true});
const physics=path.join(root,'.physics-runtime'),physicsOut=path.join(out,'vendor/mujoco');if(!fs.existsSync(path.join(physics,'runtime-manifest.json')))throw Error('Run node scripts/install-physics.mjs before building the whole-body lab.');fs.mkdirSync(physicsOut,{recursive:true});for(const file of ['mujoco.js','mujoco.wasm','runtime-manifest.json'])fs.copyFileSync(path.join(physics,file),path.join(physicsOut,file));fs.copyFileSync(path.join(root,'src/vendor/MuJoCo-LICENSE'),path.join(physicsOut,'LICENSE'));
const cfg=JSON.parse(fs.readFileSync(path.join(root,'site-config.json'),'utf8')),navigation=JSON.parse(fs.readFileSync(path.join(root,'src/navigation.json'),'utf8'));
if(navigation.at(-1).label!=='Theory')throw Error('Theory must remain the last navigation item.');
const esc=s=>String(s).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;');
const inline=s=>esc(s).replace(/`([^`]+)`/g,'<code>$1</code>').replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/\[([^\]]+)\]\(((?:https?:|[A-Za-z0-9_#])[A-Za-z0-9_:/?.=#%+&;@~-]*)\)/g,'<a href="$2">$1</a>');
const slug=s=>s.toLowerCase().replace(/[^a-z0-9\s-]/g,'').trim().replace(/\s+/g,'-');
function markdown(md){
 let result='',code=false,formula=false,list=false;const lines=md.split(/\r?\n/),closeList=()=>{if(list){result+='</ul>';list=false;}};
 for(let i=0;i<lines.length;i++){const line=lines[i];
  if(line.startsWith('```')){closeList();result+=code?'</code></pre>':'<pre class="formula"><code>';code=!code;continue;}
  if(code){result+=esc(line)+'\n';continue;}
  if(line.trim()==='\\['){closeList();result+='<pre class="formula">';formula=true;continue;}if(line.trim()==='\\]'){result+='</pre>';formula=false;continue;}
  if(formula){result+=esc(line)+'\n';continue;}
  if(line.trim().startsWith('|')&&/^\s*\|\s*:?-+/.test(lines[i+1]||'')){closeList();const cells=s=>s.trim().replace(/^\||\|$/g,'').split('|').map(v=>v.trim());result+='<table><thead><tr>'+cells(line).map(s=>'<th>'+inline(s)+'</th>').join('')+'</tr></thead><tbody>';i+=2;while(i<lines.length&&lines[i].trim().startsWith('|')){result+='<tr>'+cells(lines[i]).map(s=>'<td>'+inline(s)+'</td>').join('')+'</tr>';i++;}i--;result+='</tbody></table>';continue;}
  if(/^[-*] /.test(line)){if(!list){result+='<ul>';list=true;}result+='<li>'+inline(line.slice(2))+'</li>';continue;}
  closeList();if(!line.trim())continue;const heading=line.match(/^(#{1,4}) (.*)/);result+=heading?`<h${heading[1].length} id="${slug(heading[2])}">${inline(heading[2])}</h${heading[1].length}>`:'<p>'+inline(line)+'</p>';
 }closeList();return result;
}
const documents=[
 ['theory.html','Robot Dynamics and Coverage Guarantees','Models_and_Guarantees.md'],
 ['publication.html','Publication and Collaboration','Publication_and_Collaboration.md'],
 ['algebra-theory.html','Cyclic Operators and Topology','Theory_and_Topology.md'],
 ['comparison-theory.html','Matched Multi-Obstacle Coverage','Obstacle_Comparison.md'],
 ['structure-theory.html','Fair Structural Coverage Comparisons','Structural_Comparisons.md'],
 ['cost-theory.html','Coordination Costs and GCD','Coverage_Costs_and_GCD.md'],
 ['robot-theory.html','Enterprise Whole-Body Robot Dynamics','Enterprise_Robot_Dynamics.md'],
 ['surface-theory.html','Perforated Riemannian Surface Extension','Perforated_Surface_Extension.md'],
 ['ros2-contract.html','ROS 2 Telemetry and Adapter Contract','ROS2_Interface.md'],
 ['tiago-theory.html','TIAGo Pro Whole-Body Coverage Experiment','TIAGo_Matched_Experiment.md'],
 ['third-party.html','Third-Party Models and Runtime','Third_Party_Models.md'],
 ['Guide.html','Operating and Reproduction Guide','Running_Guide.md']
];
for(const [file,title,source]of documents){const md=fs.readFileSync(path.join(root,'docs',source),'utf8'),body=markdown(md);fs.writeFileSync(path.join(out,file),`<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>${title} | Cyclic Responsibility Lab</title><meta name="description" content="${title}: precise definitions, assumptions and reproducible research."><link rel="stylesheet" href="style.css"><link rel="stylesheet" href="site-shell.css"></head><body><main class="document"><nav></nav><p class="lab-toc"><a href="${source}" download>Download Markdown</a><a href="Theory_Center.html">Theory catalog</a></p>${body}<footer><a href="index.html">Return to the core experiment</a></footer></main></body></html>`);fs.writeFileSync(path.join(out,source),md);}
const panel=fs.readFileSync(path.join(root,'src/lab-panel.partial'),'utf8');for(const route of ['index.html','Failures.html']){const p=path.join(out,route);fs.writeFileSync(p,fs.readFileSync(p,'utf8').replace('<!-- LAB_PANEL -->',panel));}
const catalog=JSON.parse(fs.readFileSync(path.join(root,'src/theory-catalog.json'),'utf8')),theory=path.join(out,'Theory_Center.html');fs.writeFileSync(theory,fs.readFileSync(theory,'utf8').replace('<!-- THEORY_CATALOG -->',catalog.map(t=>`<a class="hub-card" href="${esc(t.href)}"><span class="status-label">${esc(t.status)}</span><h2>${esc(t.title)}</h2><p>${esc(t.description)}</p></a>`).join('')));
const guide=path.join(out,'Experiment_Guide.html');let guideText=fs.readFileSync(guide,'utf8').replaceAll('START_HERE.html','index.html').replaceAll('Distributed_Sector_Simulation.html','index.html');guideText=guideText.replace('<h1>What is each experiment testing?</h1>','<h1>What is each experiment testing?</h1><p><strong>Preserved scientific figures.</strong> These archived experiments have their original assumptions. Start at the <a href="Guide.html">current operating guide</a> for the new core dashboard, and read the model scope before comparing dynamics.</p>');
const additions=[['D01_actual_dynamics_paths','Do the actual plant trajectories differ?','Six native models use the same N=18, service k=5, fixed partition, seed and initial positions. Paths come from integrated plant states.'],['D02_physical_cost_descent','Which cost is actually decreasing?','Recorded J/k is compared with the fixed-partition centroid lower bound. No monotone envelope is plotted.'],['D03_native_control_inputs','What does each actuator actually command?','Velocity, acceleration, jerk, speed/turn, wheel-rate or generalized-force inputs use native axes.'],['D04_algebra_vs_robot_dynamics','Does a new robot model change the gcd operator?','After failure 50 to 49, orders 7 and 8 give identical mass evolution for all six models; position transients differ.']];
let extra='<h2>Multi-dynamics experiments</h2>';for(const [file,title,explanation]of additions)extra+=`<section><h2>${title}</h2><p>${explanation}</p><a href="result/${file}.png"><img loading="lazy" src="result/${file}.png" alt="${title}"></a></section>`;guideText=guideText.replace('<h2>New distributed experiments</h2>',extra+'<h2>Preserved distributed experiments</h2>');fs.writeFileSync(guide,guideText);
const routes=fs.readdirSync(out).filter(p=>p.endsWith('.html')&&!['START_HERE.html','Distributed_Sector_Simulation.html'].includes(p));
function group(route){if(route==='index.html')return 'overview';if(route==='Failures.html')return 'failures';if(['Dynamics.html','TIAGo_Comparison.html'].includes(route))return 'dynamics';if(route==='ROS2.html')return 'ros2';if(['Guide.html','publication.html'].includes(route))return 'guide';if(route.endsWith('-theory.html')||['theory.html','Theory_Center.html','ros2-contract.html'].includes(route))return 'theory';return 'comparisons';}
for(const route of routes){const p=path.join(out,route);let html=fs.readFileSync(p,'utf8'),current=group(route);const nav='<nav class="lab-nav" aria-label="Main navigation"><a class="brand" href="index.html">K-th Coverage Lab</a>'+navigation.map(item=>`<a href="${item.href}"${current===item.group?' aria-current="page"':''}>${item.label}</a>`).join('')+'</nav>';html=/<nav\b[^>]*>[\s\S]*?<\/nav>/.test(html)?html.replace(/<nav\b[^>]*>[\s\S]*?<\/nav>/,nav):(/<main\b[^>]*>/.test(html)?html.replace(/(<main\b[^>]*>)/,'$1'+nav):html.replace(/(<body\b[^>]*>)/,'$1'+nav));if(!html.includes('site-shell.css'))html=html.replace('</head>','<link rel="stylesheet" href="site-shell.css"></head>');const url=cfg.origin+'/'+(route==='index.html'?'':route);html=html.replace('</head>',`<link rel="canonical" href="${url}"><meta name="robots" content="index,follow"></head>`);if(html.includes('<!-- LAB_PANEL -->')||html.includes('<!-- THEORY_CATALOG -->'))throw Error('Unexpanded component: '+route);fs.writeFileSync(p,html);}
fs.writeFileSync(path.join(out,'robots.txt'),`User-agent: *\nAllow: /\nSitemap: ${cfg.origin}/sitemap.xml\n`);
fs.writeFileSync(path.join(out,'sitemap.xml'),'<?xml version="1.0" encoding="UTF-8"?><urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">'+routes.map(p=>`<url><loc>${cfg.origin}/${p==='index.html'?'':p}</loc></url>`).join('')+'</urlset>');
for(const alias of ['START_HERE.html','Distributed_Sector_Simulation.html'])fs.writeFileSync(path.join(out,alias),'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta http-equiv="refresh" content="0;url=index.html"><link rel="canonical" href="'+cfg.origin+'/"><title>Cyclic Responsibility Lab</title></head><body><a href="index.html">Open the current research simulator</a></body></html>');
for(const file of ['dynamics_checks.json','dynamics_case_summary.csv','obstacle_checks.json','structure_checks.json','lab_checks.json','lab_viewer_harness.json','ros2_viewer_harness.json','tiago_checks.json','tiago_python_audit.json','tiago_viewer_harness.json'])if(fs.existsSync(path.join(root,'test-results',file)))fs.copyFileSync(path.join(root,'test-results',file),path.join(out,'result',file));
fs.writeFileSync(path.join(out,'_headers'),'/\n  Cache-Control: no-cache\n/index.html\n  Cache-Control: no-cache\n/*.js\n  Cache-Control: no-cache\n');
console.log(JSON.stringify({status:'built',directory:out,routes:routes.length,origin:cfg.origin,theoryLast:true}));
