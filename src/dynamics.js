/* Actual plant equations and controllers. Native inputs retain model units.
 * No endpoint clipping or velocity reset is used to hide domain violations.
 */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.SectorDynamics=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const norm=v=>Math.hypot(...v),sat=(x,m)=>Math.max(-m,Math.min(m,x));
const MODELS=['single','double','high','unicycle','differential','lagrangian'];
function choose(n,k){let v=1;for(let j=1;j<=k;j++)v=v*(n-j+1)/j;return v;}
function metadata(c){
 const r=c.robotModel==='double'?2:c.integratorOrder;
 const common={scales:[1,1],channels:['u_x','u_y'],unit:'model units',constraint:'No pairwise collision avoidance.'};
 switch(c.robotModel){
 case 'single':return {...common,name:'Single integrator',order:1,channels:['v_x','v_y'],units:['domain units / s','domain units / s'],equation:'p_dot = u',controller:'u = a(c - p); annulus uses the feasible polar chart.',domain:c.geometry==='annulus'?'Feasible polar motion keeps the robot inside the star annulus.':'Convex-domain invariance under centroid attraction.'};
 case 'double':case 'high':return {...common,name:r===2?'Double integrator':`Order-${r} integrator`,order:r,units:[`domain units / s^${r}`,`domain units / s^${r}`],equation:`p^(${r}) = u`,controller:`Closed-loop polynomial (s + a)^${r}; initial derivatives are zero.`,domain:c.geometry==='disk'?'Convex-domain invariance for this positive filter with zero initial derivatives.':'Cartesian dynamics: annular containment is monitored, not guaranteed.'};
 case 'unicycle':return {...common,name:'Unicycle (signed speed)',order:1,channels:['v','omega'],units:['domain units / s','rad / s'],equation:'p_dot = v [cos psi, sin psi]; psi_dot = omega',controller:'v = sat(a e_parallel), omega = sat(b atan2(e_perp,e_parallel)). Reverse motion is allowed.',domain:'Nonholonomic no-slip kinematics; workspace containment is monitored, not guaranteed.'};
 case 'differential':return {...common,name:'Nonholonomic differential drive',order:1,channels:['omega_L','omega_R'],units:['rad / s','rad / s'],scales:[c.wheelSpeedLimit,c.wheelSpeedLimit],equation:'v = r(w_R + w_L)/2; psi_dot = r(w_R - w_L)/L',controller:'Projected centroid feedback converted to wheel rates; common scaling enforces wheel limits.',domain:'Same nonholonomic chassis class as the unicycle, with wheel-rate actuation.'};
 case 'lagrangian':return {...common,name:'Euler-Lagrange planar robot',order:2,channels:['tau_x','tau_y'],units:['generalized force units','generalized force units'],equation:'M(q) q_ddot + C(q,q_dot) q_dot + g(q) + D q_dot = tau',controller:'Exact-model computed torque, with a critically damped virtual acceleration.',domain:c.geometry==='disk'?'Exact cancellation gives the double-integrator positive-filter guarantee.':'Cartesian generalized coordinates: annular containment is monitored, not guaranteed.'};
 default:throw Error('Unknown robot model.');
 }
}
function validate(c){
 if(!MODELS.includes(c.robotModel))throw Error('Choose a supported robot model.');
 if(!Number.isInteger(c.integratorOrder)||c.integratorOrder<3||c.integratorOrder>5)throw Error('Higher integrator order must be 3, 4, or 5.');
 for(const key of ['robotGain','headingGain','speedLimit','turnLimit','wheelRadius','wheelBase','wheelSpeedLimit','elMass','robotStep'])if(!Number.isFinite(c[key])||c[key]<=0)throw Error(`Invalid positive dynamics parameter: ${key}.`);
 if(c.robotStep>.03||c.robotGain>3)throw Error('Use a robot substep at most 0.03 s and a pole at most 3 / s.');
 if(!Number.isFinite(c.elEccentricity)||c.elEccentricity<0||c.elEccentricity>=.8||!Number.isFinite(c.elDamping)||c.elDamping<0||!Number.isFinite(c.elGravity)||c.elGravity<0)throw Error('Invalid Euler-Lagrange parameters.');
 if(['unicycle','differential'].includes(c.robotModel)&&(c.headingGain<=c.robotGain||c.turnLimit<=c.robotGain/2))throw Error('Use heading gain > position gain and turn limit > position gain / 2.');
}
function initialize(a,c){
 const r=c.robotModel==='double'||c.robotModel==='lagrangian'?2:c.robotModel==='high'?c.integratorOrder:1;
 let y;
 if(['unicycle','differential'].includes(c.robotModel))y=[...a.p,Math.atan2(a.p[1],a.p[0])+.7*Math.sin(a.id)];
 else if(c.robotModel==='single'&&c.geometry==='annulus')y=a.nav.slice();
 else y=[...a.p,...Array(2*(r-1)).fill(0)];
 a.motion={y,order:r,input:[0,0],velocity:[0,0],heading:y.length===3?y[2]:null};
}
function mechanics(q,v,c){
 const m=c.elMass,e=c.elEccentricity,A=-m*e*Math.sin(q[1]),B=m*e*Math.cos(q[0]);
 const M=[m*(1+e*Math.cos(q[1])),m*(1+e*Math.sin(q[0]))];
 const Cv=[A*v[0]*v[1]-.5*B*v[1]**2,B*v[0]*v[1]-.5*A*v[0]**2];
 return {M,Cv,g:[0,c.elGravity*Math.sin(q[1])],A,B};
}
function chartVelocity(z,dz,geo){
 const a=z[0],u=z[1],[ri,ro]=geo.bounds(a),rip=-.24*Math.sin(2*a)+.30*Math.cos(5*a),rop=.60*Math.cos(3*a)-.10*Math.sin(a);
 const width=ro-ri-2*geo.margin,r=ri+geo.margin+u*width,ra=rip+u*(rop-rip),radial=ra*dz[0]+width*dz[1];
 return [radial*Math.cos(a)-r*Math.sin(a)*dz[0],radial*Math.sin(a)+r*Math.cos(a)*dz[0]];
}
function derivative(y,target,c,geo){
 const a=c.robotGain,model=c.robotModel;
 if(model==='single'&&c.geometry==='annulus'){
  const dy=y.map((v,j)=>a*(target[j]-v)),velocity=chartVelocity(y,dy,geo);return {dy,input:velocity,velocity,heading:null,slip:0};
 }
 if(model==='single'){const u=y.map((v,j)=>a*(target[j]-v));return {dy:u,input:u,velocity:u,heading:null,slip:0};}
 if(model==='unicycle'||model==='differential'){
  const ex=target[0]-y[0],ey=target[1]-y[1],co=Math.cos(y[2]),si=Math.sin(y[2]),parallel=ex*co+ey*si,perp=-ex*si+ey*co;
  let v=sat(a*parallel,c.speedLimit),w=sat(c.headingGain*Math.atan2(perp,parallel),c.turnLimit);
  if(Math.hypot(ex,ey)<=1e-8){v=0;w=0;} // Explicit numerical goal deadband.
  let input=[v,w];
  if(model==='differential'){
   let left=(v-c.wheelBase*w/2)/c.wheelRadius,right=(v+c.wheelBase*w/2)/c.wheelRadius;
   const scale=Math.min(1,c.wheelSpeedLimit/Math.max(Math.abs(left),Math.abs(right),1e-30));left*=scale;right*=scale;input=[left,right];
   v=c.wheelRadius*(left+right)/2;w=c.wheelRadius*(right-left)/c.wheelBase;
  }
  const velocity=[v*co,v*si];return {dy:[...velocity,w],input,velocity,heading:y[2],slip:-si*velocity[0]+co*velocity[1]};
 }
 const r=model==='lagrangian'||model==='double'?2:c.integratorOrder,dy=Array(2*r).fill(0),virtual=[0,0];
 for(let j=0;j<2;j++){
  virtual[j]=a**r*(target[j]-y[j]);for(let l=1;l<r;l++)virtual[j]-=choose(r,l)*a**(r-l)*y[2*l+j];
  for(let l=0;l<r-1;l++)dy[2*l+j]=y[2*(l+1)+j];dy[2*(r-1)+j]=virtual[j];
 }
 let input=virtual;
 if(model==='lagrangian'){
  const v=y.slice(2,4),mech=mechanics(y,v,c);
  input=virtual.map((u,j)=>mech.M[j]*u+mech.Cv[j]+mech.g[j]+c.elDamping*v[j]);
  for(let j=0;j<2;j++)dy[2+j]=(input[j]-mech.Cv[j]-mech.g[j]-c.elDamping*v[j])/mech.M[j];
 }
 return {dy,input,velocity:y.slice(2,4),heading:null,slip:0};
}
function rk4(y,dt,target,c,geo){
 const k1=derivative(y,target,c,geo).dy,k2=derivative(y.map((v,j)=>v+dt*k1[j]/2),target,c,geo).dy,k3=derivative(y.map((v,j)=>v+dt*k2[j]/2),target,c,geo).dy,k4=derivative(y.map((v,j)=>v+dt*k3[j]),target,c,geo).dy;
 return y.map((v,j)=>v+dt*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6);
}
function advance(a,goal,c,geo,dt){
 const info=metadata(c),isChart=c.robotModel==='single'&&c.geometry==='annulus';let target=goal.slice();
 if(isChart){target=geo.coordinates(goal);const reference=a.goalAngle===null?a.motion.y[0]:a.goalAngle;target[0]=reference+Math.atan2(Math.sin(target[0]-reference),Math.cos(target[0]-reference));a.goalAngle=target[0];}
 const pieces=Math.max(1,Math.ceil(dt/c.robotStep)),h=dt/pieces,points=[];
 let y=a.motion.y.slice(),d=derivative(y,target,c,geo),integral=0,peak=0,minMargin=Infinity,maxSlip=0;const before=d.input.slice();
 for(let k=0;k<pieces;k++){
  const old=d,oldP=isChart?geo.map(y):y.slice(0,2);y=rk4(y,h,target,c,geo);d=derivative(y,target,c,geo);
  if(!y.every(Number.isFinite)||norm(y)>1e8)throw Error('The chosen dynamics became numerically invalid. Reduce gains or the robot substep.');
  const n0=norm(old.input.map((u,j)=>u/info.scales[j])),n1=norm(d.input.map((u,j)=>u/info.scales[j]));integral+=(n0*n0+n1*n1)*h/2;peak=Math.max(peak,n0,n1);maxSlip=Math.max(maxSlip,Math.abs(d.slip));
  const p=isChart?geo.map(y):y.slice(0,2);minMargin=Math.min(minMargin,geo.marginAt(oldP),geo.marginAt(p));
  if(norm(p.map((v,j)=>v-oldP[j]))>1e-7)points.push(p);
 }
 a.motion={y,input:d.input.slice(),velocity:d.velocity.slice(),heading:d.heading,order:a.motion.order};a.p=isChart?geo.map(y):y.slice(0,2);if(isChart)a.nav=y.slice();
 return {beforeInput:before,afterInput:d.input.slice(),velocity:d.velocity,normalizedIntegral:integral,peak,minimumSampledMargin:minMargin,maxSlip,points};
}
return {MODELS,metadata,validate,initialize,advance,derivative,mechanics,choose,chartVelocity};
});
