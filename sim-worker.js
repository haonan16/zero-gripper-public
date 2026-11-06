import loadMujoco from './vendor/mujoco/mujoco.js';
import {makeXML,DT} from './physics.js?v=20261105-3';
let mjPromise=loadMujoco(),generation=0;
mjPromise.then(()=>postMessage({type:'ready'})).catch(e=>postMessage({type:'error',message:String(e)}));
onmessage=async({data})=>{if(data.type==='cancel'){generation++;return;}if(data.type!=='run')return;const id=++generation;try{await run(data,id);}catch(e){postMessage({type:'error',id:data.id,message:String(e?.message||e)});}};
async function run(message,token){const mj=await mjPromise,{config,left,right}=message,setup=makeXML(config,left,right);let m,d,forceBuffer;
 try{m=mj.MjModel.from_xml_string(setup.xml);d=new mj.MjData(m);const qadr=Number(m.jnt_qposadr[m.jnt('object').id]),body=m.body('object').id,close=['L','R'].map(s=>Number(m.jnt_qposadr[m.jnt('close'+s).id]));
 const ids=['L','R'].map((s,k)=>left.centers.map((_,i)=>['x','y','z'].map(a=>m.jnt(`${s}${i}${a}`).id)).flat());
 const qa=ids.map(arr=>arr.map(j=>Number(m.jnt_qposadr[j]))),da=ids.map(arr=>arr.map(j=>Number(m.jnt_dofadr[j]))),K=[left,right].map(f=>f.stiffness.flat().map(v=>v*1000)),N=qa[0].length;
 const padSets=['L','R'].map(s=>new Set(left.centers.map((_,i)=>m.body(`${s}pad${i}`).id)));
 const nA=Math.round(.7/DT),nC=Math.round(1/DT),nH=Math.round(.2/DT),nL=config.test==='close'?0:Math.ceil((config.load/(config.test==='pull'?10:50))/DT),total=nA+nC+nH+nL;
 forceBuffer=new mj.DoubleBuffer(6);const q=new Float64Array(N);let maxGrip=0,maxPenetration=0,hold=null,released=false,held=0,unstable=false;
 d.mocap_pos[2]=-.035;mj.mj_forward(m,d);
 postMessage({type:'started',id:message.id,off:setup.off,travel:setup.travel,totalSeconds:total*DT});
 const sample=(k)=>{const pos=Array.from(d.qpos.slice(qadr,qadr+3),v=>v*1000),quat=Array.from(d.qpos.slice(qadr+3,qadr+7));let grip=[0,0];let contacts=[];
 for(let j=0;j<d.ncon;j++){const c=d.contact.get(j);if(!c)continue;const b1=m.geom_bodyid[c.geom1],b2=m.geom_bodyid[c.geom2];if(b1===body||b2===body){maxPenetration=Math.max(maxPenetration,-c.dist*1000);mj.mj_contactForce(m,d,j,forceBuffer);const force=forceBuffer.GetView(),frame=c.frame,fx=frame[0]*force[0]+frame[3]*force[1]+frame[6]*force[2];for(let s=0;s<2;s++)if(padSets[s].has(b1)||padSets[s].has(b2))grip[s]+=Math.abs(fx);contacts.push(Array.from(c.pos,v=>v*1000));}c.delete();}
 const t=k*DT,load=config.test==='close'?0:config.load*Math.max(0,(k-nA-nC-nH)/Math.max(1,nL));maxGrip=Math.max(maxGrip,(grip[0]+grip[1])/2);
 if(!hold&&k>=nA+nC+nH-1)hold={pos,quat,grip};if(hold&&k>=nA+nC+nH){const movement=Math.hypot(...pos.map((v,i)=>v-hold.pos[i])),angle=2*Math.acos(Math.min(1,Math.abs(quat.reduce((v,q,i)=>v+q*hold.quat[i],0))))*180/Math.PI;if(!released&&(config.test==='pull'?movement>2:angle>10))released=true;if(!released)held=load;}
 const frame={type:'frame',id:message.id,t,progress:k/total,phase:k<nA?'approach':k<nA+nC?'close':k<nA+nC+nH?'hold':'load',grip,close:close.map(a=>d.qpos[a]*1000),pads:qa.map(arr=>arr.map(a=>d.qpos[a]*1000)),pos,quat,approach:d.mocap_pos[2]*1000,contacts,load,released,maxPenetration};
 if(pos.some(v=>!Number.isFinite(v))||qa.some(arr=>arr.some(a=>!Number.isFinite(d.qpos[a])||Math.abs(d.qpos[a])>.15))){unstable=true;throw Error('The model is unstable at this pose. Reduce closing travel or change the approach.');}
 postMessage(frame);return frame;};
 let final;
 for(let k=0;k<total;k++){if(token!==generation)return;d.mocap_pos[2]=-.035*Math.max(0,1-k/nA);d.ctrl[0]=setup.travel/1000*Math.max(0,Math.min(1,(k-nA)/nC));d.qfrc_applied.fill(0);
 for(let s=0;s<2;s++){for(let i=0;i<N;i++)q[i]=d.qpos[qa[s][i]];for(let i=0;i<N;i++){let f=0;const row=i*N;for(let j=0;j<N;j++)f+=K[s][row+j]*q[j];d.qfrc_applied[da[s][i]]=-f;}}
 const frac=Math.max(0,(k-nA-nC-nH)/Math.max(1,nL));d.xfrc_applied.fill(0);if(config.test==='pull')d.xfrc_applied[body*6+2]=config.load*frac;else if(config.test.startsWith('twist'))d.xfrc_applied[body*6+3+'xyz'.indexOf(config.test.at(-1))]=config.load*frac/1000;
 mj.mj_step(m,d);for(let ci=0;ci<d.ncon;ci++){const contact=d.contact.get(ci);maxPenetration=Math.max(maxPenetration,-contact.dist*1000);contact.delete();}if(maxPenetration>.1)throw Error('Contact overlap exceeded 0.10 mm. This run is invalid; reduce the travel or use a reference contact solve.');if(hold&&k>=nA+nC+nH){const pos=[0,1,2].map(i=>d.qpos[qadr+i]*1000),quat=[0,1,2,3].map(i=>d.qpos[qadr+3+i]);const movement=Math.hypot(...pos.map((v,i)=>v-hold.pos[i])),angle=2*Math.acos(Math.min(1,Math.abs(quat.reduce((v,q,i)=>v+q*hold.quat[i],0))))*180/Math.PI;if(config.test==='pull'?movement>2:angle>10){released=true;final=sample(k);break;}held=config.load*frac;}if(k%200===0||k===total-1){final=sample(k);if(released)break;}if(k%500===0)await new Promise(r=>setTimeout(r,0));}
 postMessage({type:'complete',id:message.id,result:{grip:hold?.grip||final.grip,maxGrip,maxPenetration,held,released,load:config.load,test:config.test,displacement:Math.hypot(...final.pos.map((v,i)=>v-[config.pose.x,config.pose.y,config.pose.height][i])),bilateral:(hold?.grip||final.grip).every(v=>v>.03),approximate:true}});
 }finally{forceBuffer?.delete();d?.delete();m?.delete();}}
