const tau=2*Math.PI;
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const turn=(p)=>((Math.atan2(p[1],p[0])/tau)%1+1)%1;
const sum=a=>a.reduce((s,v)=>s+v,0);

export function relativeRms(loads){
  const mean=sum(loads)/loads.length;
  return mean?Math.sqrt(sum(loads.map(x=>(x/mean-1)**2))/loads.length):0;
}

// The continuous background stays fixed. Two compact angular patches carry fixed
// workload shares, with the movable patch centered on the integrated object's pose.
export function createLoadField(C,base,source,parcelShare=.145,staticShare=.145){
  const field=Object.create(C.Domain.prototype);
  Object.assign(field,{obstacles:base.obstacles,A:base.A,R:base.R,rawMass:1});
  const halfWidth=.025,baseShare=1-staticShare-parcelShare;
  const hotspot={center:source.slice(0,2),angularHalfWidth:halfWidth,radius:.22,staticShare,parcelShare};
  const patch=(center,share)=>{
    const angle=turn(center)*tau,radius=Math.hypot(center[0],center[1])/10;
    const atoms=[];
    for(let i=0;i<base.A;i++){
      const lo=i*tau/base.A,hi=(i+1)*tau/base.A;
      for(const offset of [-tau,0,tau]){
        const a=Math.max(lo,angle+offset-halfWidth),b=Math.min(hi,angle+offset+halfWidth);
        if(b<=a)continue;
        const theta=(a+b)/2,p=[radius*Math.cos(theta),radius*Math.sin(theta)];
        atoms.push({p,w:share*(b-a)/(2*halfWidth),bin:i,theta,radius,patch:true});
      }
    }
    return atoms;
  };
  const background=base.samples.map(s=>({...s,w:baseShare*s.w}));
  const stationary=patch(source,staticShare);
  function update(position){
    const extra=stationary.concat(patch(position,parcelShare));
    field.samples=background.concat(extra);
    field.bins=base.bins.map(b=>({moment:b.moment.map(x=>baseShare*x)}));
    for(const s of extra){
      const v=[1,s.p[0],s.p[1],s.radius**2];
      for(let k=0;k<4;k++)field.bins[s.bin].moment[k]+=s.w*v[k];
    }
    field.cumulative=[[0,0,0,0]];
    for(const b of field.bins)field.cumulative.push(b.moment.map((x,i)=>x+field.cumulative.at(-1)[i]));
    field.total=field.cumulative.at(-1).slice();
    field.cumulative.at(-1)[0]=1;
    field.total[0]=1;
    return field;
  }
  function massAtAngle(theta){
    const u=theta/tau,whole=Math.floor(u),x=(u-whole)*field.A,index=Math.min(field.A-1,Math.floor(x));
    return whole+field.cumulative[index][0]+(x-index)*field.bins[index].moment[0];
  }
  function equalAngles(N){return Array.from({length:N+1},(_,i)=>Math.PI+i*tau/N);}
  function measure(N,k,x,referenceAgents){
    const fixed=equalAngles(N).map(massAtAngle),fixedLoads=C.sectorAssignment(field,fixed.slice(0,-1),k).moments.map(m=>m[0]);
    const angles=x.map(a=>field.inverse(a));
    const widths=angles.map((a,i)=>(i===N-1?angles[0]+tau:angles[i+1])-a);
    const optimal=bestPhase(N);
    const nearest=referenceAgents?relativeRms(C.voronoiAssignment(field,referenceAgents,k).moments.map(m=>m[0])):null;
    return {imbalance:relativeRms(fixedLoads),fixedLoads,angles,widths,distortion:Math.min(...widths)/(tau/N),
      minimumAngle:Math.min(...widths)*180/Math.PI,bestEqualMassMinAngle:optimal.angle*180/Math.PI,
      bestEqualMassRatio:optimal.angle/(tau/N),nearestFixedRobotImbalance:nearest};
  }
  function bestPhase(N){
    let best={phase:0,angle:0};
    for(let i=0;i<72;i++){
      const phase=i/(72*N),a=Array.from({length:N+1},(_,j)=>field.inverse(phase+j/N));
      const angle=Math.min(...a.slice(1).map((v,j)=>v-a[j]));
      if(angle>best.angle)best={phase,angle};
    }
    return best;
  }
  update(source);
  return {field,update,measure,hotspot,massAtAngle,bestPhase};
}

export function sourceLoadShare(hotspot,position){
  const delta=Math.atan2(Math.sin(tau*(turn(position)-turn(hotspot.center))),Math.cos(tau*(turn(position)-turn(hotspot.center))));
  return hotspot.staticShare+hotspot.parcelShare*clamp(1-Math.abs(delta)/(2*hotspot.angularHalfWidth),0,1);
}
