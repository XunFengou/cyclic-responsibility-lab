import {createAutonomousMission} from './autonomous-mission.mjs';
import {FrictionPlant} from './friction-plant.mjs';
import {createLoadField,sourceLoadShare} from './redistribution-field.mjs';

const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
const wrap=x=>Math.atan2(Math.sin(x),Math.cos(x));
const approachStages=[['clearance',2.8],['raise',3],['hover',6],['advance-high',3],['align',4]];
function ramp(x){const s=clamp(x,0,1);return s*s*s*(10+s*(-15+6*s));}

export function createRedistributionMission(modules){
  const {A,C}=modules;
  const Base=createAutonomousMission(modules,FrictionPlant).Mission;
  return class RedistributionMission extends Base {
    constructor(M,xml,method='repair',options={}){
      const cfg={mass:1,distance:2.55,duration:145,friction:.9,faultFraction:.35,parcelShare:.145,staticShare:.145,...options};
      if(!Number.isFinite(cfg.friction)||cfg.friction<.08||cfg.friction>1.6||![.35,.65].includes(cfg.faultFraction))throw Error('Use pad friction from 0.08 to 1.6 and fault progress 0.35 or 0.65.');
      if(![cfg.parcelShare,cfg.staticShare].every(s=>Number.isFinite(s)&&s>0&&s<.5)||cfg.parcelShare+cfg.staticShare>=.8)throw Error('Both demand patches must leave a positive background.');
      super(M,xml,method,cfg);
      this.originalDomain=this.domain;
      this.loadField=createLoadField(C,this.originalDomain,this.acq.pickup,cfg.parcelShare,cfg.staticShare);
      this.domain=this.loadField.field;
      this.referenceAgents=this.robots().map(r=>({id:r.id,p:r.p.slice()}));
      this.failedAcquisition=[];
      this.route=null;
      this.workflow.padId=null;
      this.workflow.targetPadId=null;
      this.workflow.planner=[];
      this.workflow.rejected=[];
      this.frames=[];
      this.capture('initial');
    }

    joints(robot){return robot.joints.map(j=>this.plant.data.qpos[j.q]);}
    stopAcquisition(robot,reason){
      const id=robot.id;
      this.acq.openGrip();
      this.acq.motion=null;
      this.controlRef=this.joints(robot);
      this.acq.braked.set(id,this.controlRef.slice());
      this.acq.failedIds.add(id);
      this.workflow.stage='quarantined';
      this.record('acquisition-stopped',{id,reason});
    }

    makeCarrierNav(){
      const domain=this.domain;
      const pads=this.acq.supports.map(p=>({x:p.pos[0]/10,y:p.pos[1]/10,r:Math.hypot(...p.halfSize)/10}));
      const parked=[...this.acq.braked.values()].map(q=>({x:q[0]/10,y:q[1]/10,r:.055}));
      const extra=pads.concat(parked);
      return new A.Navigation({obstacles:domain.obstacles.concat(extra),margin:(p,r)=>Math.min(domain.margin(p,r),...extra.map(o=>distance(p,[o.x,o.y])-o.r-r)),lineOfSight:domain.lineOfSight.bind(domain)},.045);
    }

    screenCarrierBid(robot,ik){
      const from=this.acq.restTargets.get(robot.id).slice();
      from.splice(0,3,...ik.stance,ik.theta);
      const plan=this.acq.acquisitionPlan(robot,ik.stance,ik.theta,from);
      if(!plan.okay&&this.workflow.rejected.length<24)this.workflow.rejected.push({id:robot.id,face:ik.theta,reason:plan.reason});
      return plan.okay;
    }

    phase(stage){this.workflow.stage=stage;this.phaseStart=this.time;}

    moveJoints(robot,target,duration,stage){
      this.controlRef=target.slice();
      this.acq.command(robot.id,target,duration);
      this.phase(stage);
    }

    armSettled(robot,reference,tol=.025){
      const d=this.plant.data;
      return robot.joints.every((j,i)=>i<3||i===4||i===5||Math.abs(d.qpos[j.q]-reference[i])<(i===3?.009:tol)&&Math.abs(d.qvel[j.v])<(i===3?.035:.09));
    }

    chooseDelivery(object){
      const n=this.activeIds.size,k=this.cfg.k,x=this.plant.si.x;
      const choices=this.acq.supports.filter(p=>p.id!=='source').map(pad=>{
        this.loadField.update(pad.pos);
        const m=this.loadField.measure(n,k,x,this.referenceAgents.filter(a=>this.activeIds.has(a.id)));
        return {pad,imbalance:m.imbalance,minAngle:m.bestEqualMassMinAngle,
          score:.6*m.imbalance+.4*m.nearestFixedRobotImbalance+.12*(1-m.bestEqualMassRatio)+.006*distance(pad.pos,object.pos)};
      }).sort((a,b)=>a.score-b.score||a.pad.id.localeCompare(b.pad.id));
      this.loadField.update(object.pos);
      return choices;
    }

    chooseRefuge(robot,object){
      const now=this.joints(robot),theta=this.carrierGrant.ik.theta;
      const verdicts=[];
      for(const pad of this.acq.supports.filter(p=>p.id.startsWith('refuge'))){
        const travel=distance(pad.pos,object.pos);
        const target=now.slice();
        target[0]+=pad.pos[0]-object.pos[0];
        target[1]+=pad.pos[1]-object.pos[1];
        const lineClear=this.carrierNav.clear(now.slice(0,2).map(x=>x/10),target.slice(0,2).map(x=>x/10));
        const nextTheta=theta===0?Math.PI:0;
        const stance=[pad.pos[0]-.82*Math.cos(nextTheta),pad.pos[1]];
        const reach=travel<=.52&&lineClear&&this.plant.robots.some(r=>r.id!==robot.id&&this.activeIds.has(r.id)&&!this.acq.failedIds.has(r.id)&&r.joints[3].hi>=.30&&this.robots().find(x=>x.id===r.id)?.payloadLimit>=this.cfg.mass&&this.acq.ik(r,stance,nextTheta,.15,0,{pos:pad.pos,quat:object.quat}).okay);
        verdicts.push({pad,travel,okay:reach,reason:travel>.52?'outside residual travel':!lineClear?'base route blocked':!reach?'no opposite-side replacement reach':'reachable'});
      }
      const chosen=verdicts.filter(x=>x.okay).sort((a,b)=>a.travel-b.travel)[0];
      this.workflow.planner=verdicts.map(v=>({pad:v.pad.id,travel:v.travel,okay:v.okay,reason:v.reason}));
      return chosen?.pad;
    }

    startRoute(robot,object,pad,emergency=false){
      const home=this.joints(robot),travel=distance(pad.pos,object.pos);
      this.route={home,origin:object.pos.slice(),pad,lift:emergency?0:3.5,move:Math.max(3.5,travel*(emergency?17:7)),lower:4.0,emergency};
      this.workflow.padId=pad.id;
      this.acq.motion=null;
      this.acq.mode='carry';
      this.acq.badGripTime=0;
      this.acq.loadShare=1;
      this.phase('carry');
      this.supportDwell=0;
    }

    carrierControl(){
      if(this.workflow.stage!=='carry'||!this.route)return super.carrierControl();
      const r=this.route,age=this.time-this.phaseStart,ref=r.home.slice();
      const move=ramp((age-r.lift)/r.move),lower=ramp((age-r.lift-r.move)/r.lower);
      for(let i=0;i<2;i++)ref[i]+=move*(r.pad.pos[i]-r.origin[i]);
      const rise=r.emergency?0:.133*ramp(age/r.lift);
      const fall=r.origin[2]+(r.emergency?0:.133)-r.pad.pos[2]+.017;
      ref[3]+=rise-lower*fall;
      this.acq.loadShare=1-ramp((lower-.25)/.75);
      return ref;
    }

    advance(object,dt){
      const w=this.workflow,id=w.carrierId,robot=this.plant.robots.find(r=>r.id===id),age=this.time-this.phaseStart;
      if(w.stage==='selecting'){
        if(this.cfg.relocate===false){this.phase('observe-only');return;}
        if(this.time>=this.nextCarrierBid){this.chooseCarrier();this.nextCarrierBid=this.time+2;}
        return;
      }
      if(!robot||w.stage==='complete'||w.stage==='quarantined'||w.stage==='observe-only')return;
      const q=this.joints(robot),data=this.plant.data;
      if(w.stage==='approach'){
        const target=this.carrierGrant.ik.q;
        if(distance(q,target)<.010&&Math.abs(wrap(q[2]-target[2]))<.022&&Math.hypot(...robot.joints.slice(0,2).map(j=>data.qvel[j.v]))<.045){
          this.approachPlan=this.acq.acquisitionPlan(robot,q.slice(0,2),this.carrierGrant.ik.theta,q);
          if(!this.approachPlan.okay){this.stopAcquisition(robot,this.approachPlan.reason);return;}
          this.acq.mode='align';
          this.moveJoints(robot,this.approachPlan.targets[0],approachStages[0][1],approachStages[0][0]);
        }
        return;
      }
      const approachIndex=approachStages.findIndex(([name])=>name===w.stage);
      if((approachIndex>=0||w.stage==='closing')&&this.acquisitionOrigin&&(distance(object.pos,this.acquisitionOrigin)>.034||Math.abs(object.pos[2]-this.acquisitionOrigin[2])>.026)){
        this.stopAcquisition(robot,'parcel disturbed during acquisition');return;
      }
      if(approachIndex>=0&&approachIndex<approachStages.length-1&&age>=approachStages[approachIndex][1]+.25&&this.armSettled(robot,this.controlRef,.039)&&distance(q,this.controlRef)<.016){
        const [stage,seconds]=approachStages[approachIndex+1];
        this.moveJoints(robot,this.approachPlan.targets[approachIndex+1],seconds,stage);
        if(stage==='align')this.alignmentPasses=0;
      }else if(w.stage==='align'&&age>=(this.alignmentPasses?1.6:4.3)&&robot.joints.slice(6).every(j=>Math.abs(data.qvel[j.v])<.11)){
        const targets=this.acq.handleTargets(this.carrierGrant.ik.theta);
        const worst=Math.max(...Object.entries(targets).map(([side,p])=>Math.hypot(...p.map((v,i)=>v-data.site_xpos[3*robot.sites[side]+i]))));
        if(worst>.014){
          if(this.alignmentPasses>=6){this.stopAcquisition(robot,`measured tool alignment ${worst.toFixed(3)} m`);return;}
          const ik=this.acq.ik(robot,q.slice(0,2),this.carrierGrant.ik.theta,q[3]);
          if(!ik.okay){this.stopAcquisition(robot,'fine alignment IK residual');return;}
          const ref=this.controlRef.slice();
          for(let i=6;i<q.length;i++)ref[i]=clamp(ref[i]+clamp(.75*(ik.q[i]-q[i]),-.047,.047),robot.joints[i].lo+.025,robot.joints[i].hi-.025);
          const screen=this.acq.checkSegment(robot,q,ref);
          if(!screen.okay){this.stopAcquisition(robot,'fine alignment path: '+screen.reason);return;}
          this.alignmentPasses++;
          this.moveJoints(robot,ref,1.25,'align');
          return;
        }
        this.acq.motion=null;this.acq.gripping=true;this.acq.releasing=false;this.acq.forceOverride=null;
        this.contactDwell=0;this.phase('closing');
      }else if(approachIndex>=0&&age>14){this.stopAcquisition(robot,'joint tracking gate did not settle');}
      else if(w.stage==='closing'){
        const c=object.contact,capacity=this.cfg.friction*c.normalForce;
        const certified=c.bilateral&&c.slipSpeed<.024&&capacity>1.22*this.cfg.mass*9.81&&object.speed<.035;
        this.contactDwell=certified?this.contactDwell+dt:0;
        if(this.contactDwell>=.30&&this.atomic('friction-grasp-certified',[id,this.time,c.normalForce,c.slipSpeed])){
          this.acq.contactCertificate={id,time:this.time,normalForce:c.normalForce,slipSpeed:c.slipSpeed,capacity,pads:c.pads};
          this.acq.graspHistory.push(this.acq.contactCertificate);
          this.phase('grasped');
        }else if(age>13)this.stopAcquisition(robot,'opposed-contact preload or friction margin insufficient');
      }else if(w.stage==='grasped'){
        if(!object.contact.bilateral){this.stopAcquisition(robot,'contact lost before lift');return;}
        if(!w.targetPadId){
          const choices=this.chooseDelivery(object),target=choices[0];
          if(!this.atomic('load-destination-reserved',[id,this.acq.supports.indexOf(target.pad),target.imbalance]))return;
          w.targetPadId=target.pad.id;w.deliveryChoices=choices.map(x=>({id:x.pad.id,score:x.score,imbalance:x.imbalance,minAngle:x.minAngle}));
          this.acq.goal=target.pad.pos;
        }
        if(this.beginCarry())this.startRoute(robot,object,this.acq.supports.find(p=>p.id===w.targetPadId));
      }else if(w.stage==='carry'){
        const r=this.route,progress=ramp((age-r.lift)/r.move);
        if(this.cfg.faultMode==='structural'&&!w.faultTriggered&&progress>.18){
          w.faultTriggered=true;this.acq.forceOverride=.35;
          this.record('jaw-force fault',{id,forceLimit:.35});
        }
        if(this.acq.badGripTime>.22){
          this.acq.openGrip();this.acq.motion=null;this.controlRef=q;this.acq.braked.set(id,q.slice());this.acq.failedIds.add(id);
          this.phase('quarantined');this.record('grip loss detected',{id,slip:object.peakSlip,support:object.supportId});return;
        }
        if(this.cfg.faultMode==='recoverable'&&!w.faultTriggered&&progress>=this.cfg.faultFraction){
          w.faultTriggered=true;this.acq.failedIds.add(id);this.nextAllocation=this.time;
          const pad=this.chooseRefuge(robot,object);
          this.record('degraded chassis',{id,position:object.pos,candidates:w.planner});
          if(!pad){this.controlRef=q;this.acq.motion=null;this.acq.braked.set(id,q.slice());this.phase('quarantined');this.record('no reachable safe support',{id});return;}
          this.startRoute(robot,object,pad,true);
          this.record('refuge selected',{id,pad:pad.id});return;
        }
        const arrived=age>=r.lift+r.move+r.lower&&object.stable&&object.supportId===r.pad.id;
        this.supportDwell=arrived?this.supportDwell+dt:0;
        if(this.supportDwell>=.35){
          this.controlRef=q;this.acq.jointTargets.set(id,q.slice());this.acq.openGrip();this.supportDwell=0;
          this.phase('release-check');this.record('jaws opening on measured support',{id,pad:r.pad.id,load:object.contact.supportForce});
        }else if(age>r.lift+r.move+r.lower+10){
          this.controlRef=q;this.acq.braked.set(id,q.slice());this.phase('quarantined');this.record('setdown tolerance not reached',{id,pad:r.pad.id,pos:object.pos});
        }
      }else if(w.stage==='release-check'){
        const pad=this.route.pad,opened=robot.pads.every(j=>data.qpos[j.q]<.012);
        const supported=opened&&object.stable&&object.supportId===pad.id&&object.contact.normalForce<.2;
        this.supportDwell=supported?this.supportDwell+dt:0;
        if(this.supportDwell>=.5&&this.atomic(this.route.emergency?'safe-setdown-certificate':'delivery-placed',[id,this.time,...object.pos,object.contact.supportForce])){
          const certificate={id,time:this.time,padId:pad.id,pos:object.pos,load:object.contact.supportForce,margin:object.footprintMargin};
          if(this.route.emergency)w.safeSetdowns.push(certificate);
          else{w.placementCertificate=certificate;this.jobs[1].enabled=true;this.jobs[1].progress=0;this.nextAllocation=this.time;}
          const raised=q.slice();raised[3]=.327;
          this.moveJoints(robot,raised,3,'post-release-raise');
        }else if(age>12){this.phase('quarantined');this.record('release not certified',{id,pad:pad.id});}
      }else if(w.stage==='post-release-raise'&&age>=3.4&&this.armSettled(robot,this.controlRef)){
        const back=q.slice();back[0]-=.245*Math.cos(this.carrierGrant.ik.theta);
        this.moveJoints(robot,back,4.4,'post-release-clearance');
      }else if(w.stage==='post-release-clearance'&&age>=4.7&&distance(q,this.controlRef)<.02){
        const folded=q.slice();folded.splice(6,14,...this.acq.restTargets.get(id).slice(6));
        const screen=this.acq.checkSegment(robot,q,folded);
        if(!screen.okay){this.stopAcquisition(robot,'post-release fold: '+screen.reason);return;}
        this.moveJoints(robot,folded,5,'post-release-fold');
      }else if(w.stage==='post-release-fold'&&age>=5.5&&this.armSettled(robot,this.controlRef)){
        this.acq.motion=null;this.acq.releasing=false;
        if(this.route.emergency){
          this.acq.braked.set(id,q.slice());
          this.phase('failed-retract');this.phaseStart=this.time-4;
        }else{
          this.controlRef=q;this.retreatTarget=this.acq.state().pos[0]-(this.carrierGrant.ik.theta===0?1:-1)*2.15;
          this.acq.mode='approach';this.phase('retreat');
        }
      }else if(['failed-retract','retreat','confirmation'].includes(w.stage))super.advance(object,dt);
      if(this.jobs[1].completed&&w.placementCertificate&&object.stable&&object.supportId===w.targetPadId&&!w.success){
        w.success=true;w.confirmationCertificate={time:this.jobs[1].completedAt,owners:this.postOwners.slice()};
        this.record('external confirmation',{pad:w.targetPadId});
      }
    }

    capture(phase){
      if(!this.loadField)return;
      const pos=this.acq.state().pos;
      this.loadField.update(pos);
      const frame=super.capture(phase);
      const m=this.loadField.measure(frame.row.N,this.cfg.k,frame.x,this.referenceAgents.filter(a=>this.activeIds.has(a.id)));
      Object.assign(frame.row,m,{
        transferredLoad:this.workflow.placementCertificate?this.cfg.parcelShare:0,
        sourceLoad:sourceLoadShare(this.loadField.hotspot,pos),
        normalForce:frame.object.contact.normalForce,slipSpeed:frame.object.contact.slipSpeed,
        jointLimitExcess:frame.object.maximumJointViolation,
      });
      return frame;
    }

    export(){
      return {...super.export(),schema:'friction-redistribution-v1',pads:this.acq.supports,hotspot:this.loadField.hotspot,
        equalityConstraints:this.plant.model.neq,graspHistory:this.acq.graspHistory,
        notes:['One physical parcel changes the measured spatial workload. The stationary background is conserved.',
          'All grip wrenches are MuJoCo unilateral friction contacts. No object equality constraint or pose reset is used.',
          'Recovery assumes a powered chassis with a small residual travel envelope and functioning arms.']};
    }
  };
}
