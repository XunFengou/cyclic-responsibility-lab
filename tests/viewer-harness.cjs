'use strict';
// This is an event-handler/DOM contract unit test, NOT live browser QA.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=path.resolve(__dirname,'..'),html=fs.readFileSync(path.join(root,'src/index.html'),'utf8'),source=fs.readFileSync(path.join(root,'src/viewer.js'),'utf8');
const nodes=new Map(),paintCalls=[];
const drawing=new Proxy({}, {get:(o,k)=>k in o?o[k]:(...args)=>{paintCalls.push(k);},set:(o,k,v)=>(o[k]=v,true)});
function element(id,value=''){return {id,value,checked:false,disabled:false,textContent:'',innerHTML:'',style:{},classList:{toggle(){}},parentElement:{clientWidth:630},events:{},addEventListener(t,f){this.events[t]=f;},removeAttribute(){},setAttribute(){},getContext(){return drawing;},getBoundingClientRect(){return {left:0,width:600};}};}
for(const match of html.matchAll(/<(input|select|canvas|button|div|span|h2|strong|code|p|pre)\b[^>]*\bid="([^"]+)"[^>]*>/g)){
 const [tag,type,id]=match;let value=tag.match(/\bvalue="([^"]*)"/)?.[1]||'';
 if(type==='select'){const start=html.indexOf(tag)+tag.length,options=html.slice(start,html.indexOf('</select>',start));value=options.match(/<option(?:[^>]*value="([^"]*)")?[^>]*>([^<]*)<\/option>/)?.slice(1).find(Boolean)||'';}
 const el=element(id,value);el.checked=/\bchecked\b/.test(tag);nodes.set(id,el);
}
const selected=[element('selected1'),element('selected2')],document={getElementById:id=>{assert(nodes.has(id),'Missing real DOM ID '+id);return nodes.get(id);},querySelectorAll:s=>s==='.selectedId'?selected:[],createElement:()=>element('created')};
const ctx=vm.createContext({SectorLab:require('../src/engine'),SectorDynamics:require('../src/dynamics'),document,window:{devicePixelRatio:1,addEventListener(){}},innerWidth:1380,innerHeight:1000,navigator:{clipboard:{writeText:async()=>{}}},location:{hash:'',href:'https://test.invalid/'},requestAnimationFrame:()=>1,cancelAnimationFrame(){},setTimeout(){},Blob,URL,console});
vm.runInContext(source,ctx);const models=['single','double','high','unicycle','differential','lagrangian'];
for(const model of models){nodes.get('scenario').value='fixed';nodes.get('scenario').events.change();nodes.get('model').value=model;nodes.get('model').events.change();vm.runInContext('advance(3)',ctx);assert(vm.runInContext('sim.time>0',ctx));assert(nodes.get('modelTitle').textContent.length>3);assert(nodes.get('costSummary').textContent.includes('J/k'));assert(selected[0].textContent!==undefined);}
nodes.get('N').value='1';nodes.get('N').events.change();assert(nodes.get('message').textContent.includes('between 3 and 120'));
assert(paintCalls.includes('lineTo')&&paintCalls.includes('fillText'));const result={status:'passed',scope:'Non-browser event-handler and DOM-ID contract only',modelChoicesExercised:6,chartPaintCalls:paintCalls.length,liveBrowserVerification:false};
fs.mkdirSync(path.join(root,'test-results'),{recursive:true});fs.writeFileSync(path.join(root,'test-results/viewer_harness.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
