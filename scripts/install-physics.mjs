// Official single-threaded MuJoCo. Reproducible runtime acquisition, no npm needed.
import fs from 'node:fs';import path from 'node:path';import crypto from 'node:crypto';import cp from 'node:child_process';import{fileURLToPath}from'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..'),dir=path.join(root,'.physics-runtime'),version='3.8.1';
const integrity='uIIUcdAHG48N2UBmL/iLuBoAxH1xbTs/HaJthQd5euLvk44p9ivJ5wCc+IsnqgstpCdmCZAgx2l1WGwFyxErTQ==';
fs.mkdirSync(dir,{recursive:true});const archive=path.join(dir,'package.tgz');
if(!fs.existsSync(archive)){const r=await fetch('https://registry.npmjs.org/@mujoco/mujoco/-/mujoco-'+version+'.tgz',{signal:AbortSignal.timeout(90000)});if(!r.ok)throw Error('Runtime download failed: '+r.status);fs.writeFileSync(archive,Buffer.from(await r.arrayBuffer()));}
if(crypto.createHash('sha512').update(fs.readFileSync(archive)).digest('base64')!==integrity)throw Error('MuJoCo package integrity mismatch');
cp.execFileSync('tar',['-xzf',archive,'-C',dir,'--strip-components=1','package/mujoco.js','package/mujoco.wasm','package/mujoco.d.ts','package/package.json']);
const manifest={package:'@mujoco/mujoco',version,integrity:'sha512-'+integrity,files:Object.fromEntries(['mujoco.js','mujoco.wasm'].map(n=>[n,crypto.createHash('sha256').update(fs.readFileSync(path.join(dir,n))).digest('hex')]))};fs.writeFileSync(path.join(dir,'runtime-manifest.json'),JSON.stringify(manifest,null,2));console.log(JSON.stringify({status:'ready',...manifest}));
