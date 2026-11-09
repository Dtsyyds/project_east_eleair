/* Live sensor-frame point cloud. Presentation only; never sends ROS commands. */
"use strict";
(() => {
  const canvas=document.getElementById("lidar-preview"), ctx=canvas.getContext("2d");
  const status=document.getElementById("lidar-preview-status"), meta=document.getElementById("lidar-preview-meta");
  const model=window.VehiclePreviewModel||null;
  const defaultRange=model?1.25:10;
  let showVehicle=true, levelVehicle=true, renderedTriangles=0;
  let cloud=null, arrival=0, received=0, displayed=0, invalid=0, connected=false, connection=null;
  let angle=.65, tilt=.75, range=defaultRange, dragging=null, dirty=true, scheduled=false, lastFresh=false;
  const times=[];
  function schedule() {
    if(scheduled)return;scheduled=true;
    requestAnimationFrame(()=>{scheduled=false;draw();});
  }
  function receive(value) {
    if(!connected)return;
    if(!value || value.error || value.position_unit!=="m" || !/^\d+$/.test(value.stamp_ns||"") ||
       typeof value.frame_id!=="string" || !Array.isArray(value.points) || value.points.length>2000 ||
       !Number.isInteger(value.source_points) || value.source_points<value.points.length ||
       value.points.some(p=>!Array.isArray(p)||p.length!==3||p.some(v=>!Number.isFinite(v)||Math.abs(v)>1e6))) {
      invalid++;return;
    }
    if(cloud && cloud.frame_id===value.frame_id && BigInt(value.stamp_ns)<=BigInt(cloud.stamp_ns))return;
    cloud=value;arrival=performance.now();received++;times.push(arrival);
    while(times.length && times[0]<arrival-2000)times.shift();
    dirty=true;schedule();
  }
  function update(isConnected, id) {
    if(id!==connection || (!isConnected && connected)) {
      cloud=null;arrival=0;times.length=0;dirty=true;connection=id;
    }
    connected=isConnected;
    const fresh=connected && !!cloud && performance.now()-arrival<2500;
    if(fresh!==lastFresh){dirty=true;lastFresh=fresh;}
    status.textContent=!connected?"连接已断开":!cloud?"等待点云":!fresh?"点云已过期":cloud.points.length?"实时点云":"本帧无有效点";
    status.className="badge "+(fresh?"good":"warn");
    if(cloud) {
      const hz=times.length>1?(times.length-1)*1000/(times.at(-1)-times[0]):0;
      meta.textContent=`${cloud.frame_id} ${levelVehicle&&modelApplicable()?"→ 车体水平（估计）":""} · 米 · 原始 ${cloud.source_points} 点 / 预览 ${cloud.points.length} 点 · ${fresh?hz.toFixed(1):"0.0"} FPS · 源时间 ${cloud.stamp_ns}`;
    } else meta.textContent="等待 /livox/lidar；显示雷达自身坐标系的单帧点云。";
    if(dirty)schedule();
  }
  function modelApplicable() {return !!model && (!cloud || cloud.frame_id===model.source_frame);}
  function rotate(p,R) {return R.map(row=>row[0]*p[0]+row[1]*p[1]+row[2]*p[2]);}
  function cloudPoint(p) {
    if(!levelVehicle||!modelApplicable())return p;
    return rotate(p,model.rotation).map((v,i)=>v+model.translation_m[i]);
  }
  function vehiclePoint(p) {
    if(levelVehicle)return p;
    const q=p.map((v,i)=>v-model.translation_m[i]);
    return [0,1,2].map(i=>model.rotation[0][i]*q[0]+model.rotation[1][i]*q[1]+model.rotation[2][i]*q[2]);
  }
  function draw() {
    if(document.hidden || !canvas.getClientRects().length)return;
    dirty=false;
    const ratio=Math.min(2,window.devicePixelRatio||1),w=Math.max(1,canvas.clientWidth),h=Math.max(1,canvas.clientHeight);
    canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);
    ctx.fillStyle="#101b29";ctx.fillRect(0,0,w,h);
    const scale=Math.min(w,h)/(2*range),ca=Math.cos(angle),sa=Math.sin(angle),ct=Math.cos(tilt),st=Math.sin(tilt);
    const project=p=>[w/2+(-p[0]*sa+p[1]*ca)*scale,h/2-((p[0]*ca+p[1]*sa)*st+p[2]*ct)*scale];
    function line(a,b,color,label) {
      const p=project(a),q=project(b);ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();
      if(label){ctx.fillStyle=color;ctx.fillText(label,q[0]+4,q[1]);}
    }
    const depth=p=>ct*(p[0]*ca+p[1]*sa)-st*p[2];
    const ground=levelVehicle&&modelApplicable()?(model.ground_z_m??-.047):0;
    const step=Math.pow(10,Math.floor(Math.log10(range/3)));
    for(let x=-range;x<=range;x+=step){line([x,-range,ground],[x,range,ground],"#26394d");line([-range,x,ground],[range,x,ground],"#26394d");}
    ctx.font="13px sans-serif";line([0,0,0],[range*.7,0,0],"#ef7777","X");line([0,0,0],[0,range*.7,0],"#7ad6a0","Y");line([0,0,0],[0,0,range*.7],"#7abaff","Z");
    const fresh=connected&&cloud&&performance.now()-arrival<2500;
    const objects=[];renderedTriangles=0;
    if(cloud) {
      for(const original of cloud.points) {
        const p=cloudPoint(original),q=project(p);
        if(q[0]<0||q[0]>w||q[1]<0||q[1]>h)continue;
        objects.push({depth:depth(p),point:q,color:fresh?`hsl(${Math.max(0,Math.min(240,180-p[2]*30))} 80% 65%)`:"#566576"});
      }
      displayed++;
    }
    if(showVehicle&&modelApplicable()) {
      for(const part of model.parts) {
        const vertices=part.vertices.map(vehiclePoint),projected=vertices.map(project),depths=vertices.map(depth);
        for(const face of part.faces) {
          const points=face.map(i=>projected[i]);
          if(points.every(p=>p[0]<0)||points.every(p=>p[0]>w)||points.every(p=>p[1]<0)||points.every(p=>p[1]>h))continue;
          const [a,b,c]=face.map(i=>vertices[i]);
          const u=b.map((v,i)=>v-a[i]),v=c.map((v,i)=>v-a[i]);
          const normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
          const length=Math.hypot(...normal);if(length<1e-10)continue;
          const light=.4+.6*Math.abs((-.3*normal[0]-.4*normal[1]+.866*normal[2])/length);
          objects.push({depth:face.reduce((a,i)=>a+depths[i],0)/3,triangle:points,
                        color:`rgb(${part.color.map(c=>Math.round(c*light)).join(",")})`});
          renderedTriangles++;
        }
      }
    }
    objects.sort((a,b)=>b.depth-a.depth);
    for(const object of objects) {
      ctx.fillStyle=object.color;
      if(object.point)ctx.fillRect(object.point[0],object.point[1],2,2);
      else {ctx.beginPath();ctx.moveTo(...object.triangle[0]);ctx.lineTo(...object.triangle[1]);ctx.lineTo(...object.triangle[2]);ctx.closePath();ctx.fill();}
    }
    if(showVehicle&&modelApplicable()) {
      const q=project(levelVehicle?model.translation_m:[0,0,0]);
      ctx.fillStyle="#ffb55b";ctx.beginPath();ctx.arc(q[0],q[1],3,0,Math.PI*2);ctx.fill();ctx.fillText("雷达",q[0]+7,q[1]-5);
      ctx.fillStyle="#dfb86d";ctx.fillText(`${model.label||"车体"} · ${levelVehicle?"水平显示（估计）":"原始雷达坐标"}`,12,20);
    }
    ctx.fillStyle="#c7d7e7";ctx.fillText(`网格 ${step} m · 视野 ±${range.toFixed(1)} m`,12,h-14);
  }
  canvas.addEventListener("pointerdown",e=>{dragging=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener("pointermove",e=>{if(!dragging)return;angle+=(e.clientX-dragging[0])*.008;tilt=Math.max(-1.55,Math.min(1.55,tilt+(e.clientY-dragging[1])*.008));dragging=[e.clientX,e.clientY];dirty=true;schedule();});
  for(const event of ["pointerup","pointercancel","lostpointercapture"])canvas.addEventListener(event,()=>{dragging=null;});
  canvas.addEventListener("wheel",e=>{e.preventDefault();range=Math.max(.5,Math.min(200,range*Math.exp(e.deltaY*.001)));dirty=true;schedule();},{passive:false});
  document.getElementById("lidar-preview-reset").onclick=()=>{angle=.65;tilt=.75;range=defaultRange;dirty=true;schedule();};
  document.getElementById("lidar-preview-top").onclick=()=>{angle=-Math.PI/2;tilt=Math.PI/2;dirty=true;schedule();};
  document.getElementById("lidar-vehicle-show")?.addEventListener("change",e=>{showVehicle=e.target.checked;dirty=true;schedule();});
  document.getElementById("lidar-vehicle-level")?.addEventListener("change",e=>{levelVehicle=e.target.checked;dirty=true;schedule();});
  document.getElementById("lidar-vehicle-close")?.addEventListener("click",()=>{range=.6;dirty=true;schedule();});
  const estimate=document.getElementById("lidar-vehicle-estimate");
  if(estimate&&model)estimate.textContent=`雷达中心估计：X ${model.translation_m[0].toFixed(3)} / Y ${model.translation_m[1].toFixed(3)} / Z ${model.translation_m[2].toFixed(3)} m；按静止 IMU 调平约 ${model.tilt_degrees.toFixed(1)}°。${model.model_id?`CAD 雷达轴到 livox_frame 的对应为假设，CAD 与 IMU 差 ${model.cad_imu_residual_degrees.toFixed(1)}°；原点为模型底面中心。`:"朝向沿用 CAD 假设。"}非实测外参。`;
  new ResizeObserver(()=>{dirty=true;schedule();}).observe(canvas);
  window.LidarPreview=Object.freeze({receive,update,snapshot:()=>({vehicle_model_loaded:!!model,vehicle_model_id:model?.model_id||null,vehicle_triangles:renderedTriangles,vehicle_visible:showVehicle,level_display:levelVehicle,display_transform_estimated:!!model,range_m:range,received,displayed,invalid,points:cloud?.points.length||0,source_points:cloud?.source_points||0,frame_id:cloud?.frame_id,stamp_ns:cloud?.stamp_ns,fresh:connected&&!!cloud&&performance.now()-arrival<2500})});
})();
