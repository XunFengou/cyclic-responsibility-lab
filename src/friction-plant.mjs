import {AcquisitionPlant} from './acquisition-plant.mjs';

const clamp=(x,lo,hi)=>Math.max(lo,Math.min(hi,x));
const dot=(a,b)=>a.reduce((sum,x,i)=>sum+x*b[i],0);
const sides=['left','right'];

export const loadPads=Object.freeze([
  {id:'source',pos:[-5.2,-2,.95],halfSize:[.205,.192]},
  {id:'refuge-a',pos:[-5.2,-1.15,.95],halfSize:[.205,.192]},
  {id:'refuge-b',pos:[-5.2,-.35,.95],halfSize:[.205,.192]},
  {id:'destination',pos:[-5.2,.55,.95],halfSize:[.205,.192]},
]);

function fixture(friction){
  return {
    pickup:loadPads[0].pos,
    goal:loadPads.at(-1).pos,
    supportsXml:loadPads.map((p,i)=>`<geom name="load_pad_${i}" type="box" pos="${p.pos[0]} ${p.pos[1]} .875" size="${p.halfSize.join(' ')} .05" contype="1" conaffinity="7" rgba="${i===0?'.68 .32 .19':i===3?'.20 .57 .39':'.24 .44 .59'} 1" friction=".95 .02 .001"/>`).join(''),
    configureXml(xml){
      const names=[...xml.matchAll(/name="(r\d+_(?:left|right)_pad_(?:plus|minus)_geom)"/g)].map(x=>x[1]);
      const pairs=names.flatMap(name=>[0,1].map(h=>`<pair geom1="${name}" geom2="handle_${h}" condim="4" friction="${friction} ${friction} .015 .001 .001" solref=".013 1" solreffriction=".010 1" solimp=".999 .9999 .001"/>`));
      return xml.replace('<option ','<option cone="elliptic" ')
        .replace('</contact>',pairs.join('')+'</contact>')
        .replace(/<geom\b[^>]+name="(?:r\d+_(?:left|right)_pad_(?:plus|minus)_geom|handle_[01])"[^>]*\/>/g,geom=>
          geom.replace(/friction="[^"]+"/,`condim="4" friction="${friction} .015 .001"`)
            .replace('size=".008 .022 .018"','size=".008 .029 .030"').replace('solref=".008 1"','solref=".014 1"'));
    },
  };
}

export class FrictionPlant extends AcquisitionPlant {
  constructor(p,modules,options){
    const friction=options.friction??.9;
    super(p,modules,options,fixture(friction));
    if(p.model.neq!==0)throw Error('The friction experiment requires a scene without equality constraints.');
    this.friction=friction;
    this.forceTarget=Math.min(22,2.35*options.mass*9.81/(4*friction));
    this.approachSpeed=.32;
    this.gripping=false;
    this.releasing=false;
    this.loadShare=0;
    this.filteredNormal=new Map();
    this.armTrim=new Map();
    this.supports=loadPads;
    this.supportGeoms=new Map(loadPads.map((p,i)=>[this.M.mj_name2id(this.p.model,this.M.mjtObj.mjOBJ_GEOM.value,`load_pad_${i}`),p]));
    this.geomNames=Array.from({length:p.model.ngeom},(_,id)=>this.M.mj_id2name(p.model,this.M.mjtObj.mjOBJ_GEOM.value,id)||'');
    this.peakSlip=0;
    this.slipDistance=0;
    this.badGripTime=0;
    this.peakNormal=0;
    this.graspHistory=[];
    for(const r of p.robots)for(const pad of r.pads)p.data.qpos[pad.q]=.003;
    this.M.mj_forward(p.model,p.data);
    this.initialQpos=Array.from(p.data.qpos);
  }

  carrierGain(index){return index<3?28:index>=6?230:110;}

  jointFrictionCompensation(j,error){
    const velocity=this.p.data.qvel[j.v],q=this.p.data.qpos[j.q];
    const previous=this.armTrim.get(j.v)||0;
    const trim=Math.abs(velocity)<.14?clamp(previous+.005*4*error,-1.6,1.6):previous*.999;
    this.armTrim.set(j.v,trim);
    const torque=1.04*this.p.model.dof_frictionloss[j.v]*Math.tanh(160*error)+clamp(8*error+trim-.32*velocity,-2.0,2.0);
    return clamp(torque,q<j.lo+.035?0:-4,q>j.hi-.035?0:4);
  }

  limitAcceleration(ades){
    const {data,model,robots}=this.p;
    for(const robot of robots)for(const [index,j] of robot.joints.entries()){
      if(!model.jnt_limited[j.j])continue;
      const slide=index===3,brake=slide?.75:12;
      const speed=slide?.12:1.2,limit=slide?1.2:50;
      const margin=slide?.006:.025;
      const q=data.qpos[j.q],v=data.qvel[j.v];
      const hi=Math.min(speed,Math.sqrt(2*brake*Math.max(0,j.hi-margin-q)));
      const lo=-Math.min(speed,Math.sqrt(2*brake*Math.max(0,q-j.lo-margin)));
      ades[j.v]=clamp((clamp(v+.005*ades[j.v],lo,hi)-v)/.005,-limit,limit);
    }
  }

  jawAcceleration(robot,pad){
    if(this.gripping&&robot.id===this.selected)return 0;
    return clamp(2800*(.003-this.p.data.qpos[pad.q])-105*this.p.data.qvel[pad.v],-90,90);
  }

  jawCommand(robot,pad){
    if(!this.gripping||robot.id!==this.selected)return null;
    const {data}=this.p,normal=this.filteredNormal.get(pad.geom)||0;
    const target=this.forceOverride??this.forceTarget;
    if(normal<.35){
      if(data.qpos[pad.q]>.0475)return 0;
      return clamp(1.1-28*data.qvel[pad.v],-.8,2);
    }
    return clamp(target+.4*(target-normal)-12*data.qvel[pad.v],0,28);
  }

  loadBearingHands(){return this.gripping&&this.lastContact?.bilateral?sides:[];}

  latch(){throw Error('Equality-based grasp is disabled.');}

  openGrip(){
    this.gripping=false;
    this.releasing=true;
    this.loadShare=0;
  }

  contacts(id=this.selected){
    const {M,p}=this,{model:m,data:d}=p;
    const robot=p.robots.find(r=>r.id===id),pads=[],supports=new Map();
    const handForces={left:[0,0,0],right:[0,0,0]};
    let minimum=0,maxSlip=0,maxUtilization=0;
    const contacts=d.contact;
    try{
      for(let index=0;index<contacts.size();index++){
        const c=contacts.get(index);
        try{
        const g0=c.geom[0],g1=c.geom[1];
        minimum=Math.min(minimum,c.dist);
        if(c.efc_address<0)continue;
        const pad=robot?.pads.find(pad=>pad.geom===g0||pad.geom===g1);
        const handle=pad&&this.handles.find(h=>h===g0||h===g1);
        const supportGeom=this.supportGeoms.has(g0)?g0:this.supportGeoms.has(g1)?g1:null;
        const onSupport=supportGeom!==null&&m.geom_bodyid[supportGeom===g0?g1:g0]===this.tray;
        if(handle===undefined&&!onSupport)continue;
        M.mj_contactForce(m,d,index,this.cf);
        const f=this.cf.GetView(),frame=c.frame;
        if(onSupport){
          const key=this.supportGeoms.get(supportGeom).id,sign=supportGeom===g0?1:-1;
          const up=sign*(f[0]*frame[2]+f[1]*frame[5]+f[2]*frame[8]);
          supports.set(key,(supports.get(key)||0)+Math.max(0,up));
        }
        if(handle!==undefined&&f[0]>.06){
          const sign=pad.geom===g0?1:-1;
          const slip=Math.hypot(d.efc_vel[c.efc_address+1],d.efc_vel[c.efc_address+2]);
          const utilization=Math.hypot(f[1]/c.friction[0],f[2]/c.friction[1],f[3]/c.friction[2])/f[0];
          const normal=[frame[0]*sign,frame[1]*sign,frame[2]*sign];
          for(let axis=0;axis<3;axis++)handForces[pad.side][axis]+=sign*(f[0]*frame[axis]+f[1]*frame[axis+3]+f[2]*frame[axis+6]);
          pads.push({geom:pad.geom,side:pad.side,sign:pad.sign,force:f[0],normal,handle,slip,utilization});
          maxSlip=Math.max(maxSlip,slip);
          maxUtilization=Math.max(maxUtilization,utilization);
        }
        }finally{c.delete();}
      }
    }finally{contacts.delete();}
    const pairs=sides.map(side=>{
      const plus=pads.find(p=>p.side===side&&p.sign==='plus'&&p.force>.8);
      const minus=pads.find(p=>p.side===side&&p.sign==='minus'&&p.force>.8);
      return plus&&minus&&plus.handle===minus.handle&&dot(plus.normal,minus.normal)<-.8?plus.handle:null;
    });
    const bearing=[...supports].sort((a,b)=>b[1]-a[1])[0];
    return {minimum,support:!!bearing,supportId:bearing?.[0]??null,supportForce:bearing?.[1]??0,pads,
      bilateral:pairs.every(x=>x!==null)&&pairs[0]!==pairs[1],handForces,slipSpeed:maxSlip,utilization:maxUtilization,
      normalForce:pads.reduce((s,p)=>s+p.force,0)};
  }

  monitorContacts(contact){
    this.lastContact=contact;
    const robot=this.p.robots.find(r=>r.id===this.selected);
    if(!robot)return;
    for(const pad of robot.pads){
      const normal=contact.pads.filter(p=>p.geom===pad.geom).reduce((s,p)=>s+p.force,0);
      this.filteredNormal.set(pad.geom,.82*(this.filteredNormal.get(pad.geom)||0)+.18*normal);
    }
    if(!this.gripping)return;
    this.peakSlip=Math.max(this.peakSlip,contact.slipSpeed);
    this.peakNormal=Math.max(this.peakNormal,contact.normalForce);
    if(this.mode==='carry'){
      this.slipDistance+=.005*contact.slipSpeed;
      this.badGripTime=(!contact.bilateral||contact.slipSpeed>.065)?this.badGripTime+.005:0;
    }
  }

  footprintMargin(pad,state){
    return Math.min(...pad.halfSize.map((a,i)=>a-Math.abs(pad.pos[i]-state.pos[i])));
  }

  frame(){
    const o=this.state(),contact=this.contacts(),r=this.p.robots.find(r=>r.id===this.selected);
    const tilt=Math.acos(clamp(1-2*(o.quat[1]**2+o.quat[2]**2),-1,1)),speed=Math.hypot(...o.velocity.slice(0,3));
    const pad=this.supports.find(p=>p.id===contact.supportId),margin=pad?this.footprintMargin(pad,o):null;
    const stable=!!pad&&margin>.135&&contact.supportForce>.84*this.cfg.mass*9.81&&speed<.035&&tilt<.085&&Math.abs(o.pos[2]-pad.pos[2])<.018;
    return {...o,contact,carrierId:this.selected,supportId:contact.supportId,footprintMargin:margin,stable,speed,tilt,
      height:Math.abs(o.pos[2]-.95),xy:Math.hypot(o.pos[0]-this.goal[0],o.pos[1]-this.goal[1]),
      forces:contact.handForces,activeGrasps:sides.map(side=>contact.pads.filter(p=>p.side===side&&p.force>.8).length>=2),
      minimumContactDistance:this.minContact,maximumJointViolation:this.maxJointViolation,pickup:this.pickup,goal:this.goal,
      peakSlip:this.peakSlip,slipDistance:this.slipDistance,forceTarget:this.gripping?(this.forceOverride??this.forceTarget):0,
      normalCommand:r?r.pads.map(p=>this.p.data.ctrl[p.a]):[],nq:this.p.model.nq,nv:this.p.model.nv,links:[]};
  }

  checkSegment(robot,from,to){
    const {p,M,scratch:d}=this,prefix=`r${robot.id}_`;
    d.qpos.set(p.data.qpos);
    d.qvel.fill(0);
    for(const jaw of robot.pads)d.qpos[jaw.q]=.003;
    const yawDelta=Math.atan2(Math.sin(to[2]-from[2]),Math.cos(to[2]-from[2]));
    for(let step=0;step<=20;step++){
      const a=step/20;
      for(const [i,j] of robot.joints.entries()){
        const value=from[i]+a*(i===2?yawDelta:to[i]-from[i]);
        if(p.model.jnt_limited[j.j]&&(value<j.lo+.004||value>j.hi-.004))return {okay:false,reason:j.name+' margin',value};
        d.qpos[j.q]=value;
      }
      M.mj_fwdPosition(p.model,d);
      const contacts=d.contact;
      try{
        for(let n=0;n<contacts.size();n++){
          const c=contacts.get(n);
          try{
            const names=[this.geomNames[c.geom[0]],this.geomNames[c.geom[1]]];
            if(c.dist<-.0015&&names.some(name=>name.startsWith(prefix)))return {okay:false,reason:names.join(' / '),distance:c.dist,at:a};
          }finally{c.delete();}
        }
      }finally{contacts.delete();}
    }
    return {okay:true};
  }

  acquisitionPlan(robot,stance,theta,from,pose){
    const contact=this.ik(robot,stance,theta,.15,0,pose);
    if(!contact.okay)return {okay:false,reason:'contact IK residual'};
    const back=from.slice();back[0]-=.245*Math.cos(theta);
    const raised=back.slice();raised[3]=.327;
    const hoverIn=contact.q.slice();hoverIn[3]=raised[3];
    const hoverOut=hoverIn.slice();hoverOut[0]=back[0];
    const targets=[back,raised,hoverOut,hoverIn,contact.q];
    let start=from;
    for(const target of targets){
      const result=this.checkSegment(robot,start,target);
      if(!result.okay)return result;
      start=target;
    }
    return {okay:true,targets};
  }
}
