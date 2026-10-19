/* Sensor playback panels. No framework, external assets or live-topic mixing. */
"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const COLORS = ["#70ddb1", "#8fcaff", "#ffd080", "#ff9393", "#bc9fff", "#ffab76", "#70dce0", "#eeeeee"];
  const LABELS = {idle:"未回放",starting:"正在打开",playing:"正在回放",paused:"已暂停",stopping:"正在停止",stopped:"已停止",complete:"回放结束",failed:"回放失败"};
  const SPECS = [
    ["mindvision","工业相机","image"],["fisheye","鱼眼相机","image"],
    ["d405_color","D405 彩色","image"],["d405_depth","D405 深度伪彩","image"],
    ["d405_infra1","D405 红外 1","image"],["d405_infra2","D405 红外 2","image"],
    ["lidar","MID-360 点云","cloud"],["imu","IMU","imu"],["imu_raw","IMU 原始数据","imu"],
    ["eddy_raw","涡流原始波形","waveform"],["eddy_mean","涡流通道均值","means"],
    ["emat_raw","电磁超声波形","waveform"],["emat_result","测厚结果","result"]
  ];
  let api, connected=false, replay=null, session=null, epoch=0, catalog=[], panels=new Map(), topics=new Map(), failure="";
  let raf=null, angle=.7, tilt=.5, zoom=1, dragging=null;
  const text = value => value == null ? "—" : String(value);
  const fixed = v => typeof v === "number"&&Number.isFinite(v)?v.toFixed(3):"—";

  function clearPanels() {
    epoch++; session=null;panels.clear();topics.clear();$("replay-windows").replaceChildren();$("replay-other-topics").replaceChildren();
  }

  function buildWindows(value) {
    const descriptors=new Map((value.sensors||[]).map(s=>[s.id,s]));
    for(const [id,label,kind] of SPECS) {
      const spec=descriptors.get(id)||{count:0};
      const window=document.createElement("section");window.className="panel replay-window";window.id="replay-window-"+id;
      const head=document.createElement("div");head.className="head";
      const title=document.createElement("h2");title.textContent=label;
      const button=document.createElement("button");button.textContent="全屏";button.className="quiet";
      button.onclick=()=>window.requestFullscreen?.();head.append(title,button);
      const empty=document.createElement("div");empty.className="replay-empty";empty.textContent=spec.count>0?"等待本传感器的记录…":"本批无数据";
      const canvas=document.createElement("canvas");canvas.id="replay-canvas-"+id;canvas.width=640;canvas.height=300;canvas.hidden=!(spec.count>0)||kind==="result";
      const values=document.createElement("p");values.className="replay-values";
      const meta=document.createElement("p");meta.className="replay-meta";meta.textContent=(spec.topic||"该批次未配置此话题")+"\n记录数量："+text(spec.count||0);
      window.append(head,empty,canvas,values,meta);
      if(kind==="result") {const body=document.createElement("pre");body.textContent="尚无测厚结果";window.append(body);}
      $("replay-windows").append(window);
      panels.set(id,{id,kind,spec,window,canvas,empty,values,meta,received:0,displayed:0,errors:0,version:0,busy:false,pending:null,data:null,history:[],dirty:false});
    }
    const cloud=panels.get("lidar").canvas;
    cloud.addEventListener("pointerdown",event=>{dragging={x:event.clientX,y:event.clientY};cloud.setPointerCapture(event.pointerId);});
    cloud.addEventListener("pointermove",event=>{if(!dragging)return;angle+=(event.clientX-dragging.x)*.008;tilt=Math.max(-1.5,Math.min(1.5,tilt+(event.clientY-dragging.y)*.008));dragging={x:event.clientX,y:event.clientY};panels.get("lidar").dirty=true;schedule();});
    cloud.addEventListener("pointerup",()=>{dragging=null;});cloud.addEventListener("pointercancel",()=>{dragging=null;});
    cloud.addEventListener("wheel",event=>{event.preventDefault();zoom=Math.max(.1,Math.min(20,zoom*Math.exp(-event.deltaY*.001)));panels.get("lidar").dirty=true;schedule();},{passive:false});
    const known=new Set((value.sensors||[]).map(s=>s.topic));
    for(const item of value.topics||[]) {
      const details=document.createElement("details"),summary=document.createElement("summary"),pre=document.createElement("pre");
      summary.textContent=item.topic+" · "+item.count+" 条"+(known.has(item.topic)?" · 已在窗口显示":"");
      pre.textContent=item.count>0?"等待回放该话题…":"本批无数据";details.append(summary,pre);$("replay-other-topics").append(details);
      topics.set(item.topic,{pre,summary,spec:item});
    }
  }

  function history(items) {
    const selected=$("replay-run").value;
    catalog=(Array.isArray(items)?items:[]).filter(x=>/^WS-[A-Za-z0-9_-]+$/.test(x?.run_id||"")&&["complete","incomplete","failed"].includes(x.status));
    const options=catalog.map(item=>{const option=document.createElement("option");option.value=item.run_id;option.textContent=item.run_id+(["incomplete","failed"].includes(item.status)?"（未完整，回放前检查文件）":"");return option;});
    if(!options.length){const option=document.createElement("option");option.value="";option.textContent="尚无已关闭的采集批次";options.push(option);}
    $("replay-run").replaceChildren(...options);
    if(catalog.some(x=>x.run_id===selected))$("replay-run").value=selected;
    render();
  }

  function status(value) {
    if(!value||typeof value!=="object")return;
    if(value.service_id===replay?.service_id&&Number.isFinite(value.revision)&&Number.isFinite(replay?.revision)&&value.revision<replay.revision)return;
    replay=value;
    if(value.session_id&&value.session_id!==session) {
      clearPanels();session=value.session_id;angle=.7;tilt=.5;zoom=1;buildWindows(value);
    }
    render();
  }

  function receive(packet) {
    const payload=packet.payload;
    if(packet.header.msg_type==="response"&&payload.action?.startsWith("replay.")&&payload.status==="failed")failure=payload.error||"回放操作失败";
    if(Array.isArray(payload.history))history(payload.history);
    if(payload.replay)status(payload.replay);
    if(packet.header.msg_type!=="replay_sensor"||payload.session_id!==session)return;
    const panel=panels.get(payload.sensor_id),topic=topics.get(payload.topic),data=payload.data;
    if(!data||typeof data!=="object")return;
    if(topic) {topic.pre.textContent=JSON.stringify(data.kind==="generic"?data.message:data.kind==="error"?data:{kind:data.kind,stamp_ns:payload.stamp_ns,frame_id:payload.frame_id},null,2);}
    if(!panel)return;
    if(payload.timestamp_ns&&panel.latest?.timestamp_ns) {
      try {if(BigInt(payload.timestamp_ns)<=BigInt(panel.latest.timestamp_ns))return;} catch(_) {return;}
    }
    panel.received++;panel.latest=payload;
    if(data.kind==="error") {panel.errors++;panel.empty.hidden=false;panel.empty.textContent="预览错误："+text(data.message);return;}
    panel.data=data;
    panel.meta.textContent=payload.topic+"\n源时间："+text(payload.stamp_ns)+" · 坐标系："+text(payload.frame_id)+"\n记录 "+panel.spec.count+" 条 · 收到预览 "+panel.received+" 帧 · 回放 "+fixed(payload.position_seconds)+" 秒";
    if(data.kind==="image") {panel.pending=data;decode(panel);return;}
    if(data.kind==="means"||data.kind==="imu") {
      const values=data.kind==="means"?data.values:[...(data.acceleration||[]),...(data.gyro||[])];
      panel.history.push(values);if(panel.history.length>300)panel.history.shift();
    }
    panel.empty.hidden=true;panel.displayed++;
    if(data.kind==="result") {
      const result=data.result||{};panel.window.querySelector("pre").textContent=JSON.stringify(result,null,2);
      panel.values.textContent=Number.isFinite(result.thickness_mm)?"厚度："+result.thickness_mm.toFixed(4)+" mm · "+(result.source==="saved"?"已有波形":"实时采集"):"记录未提供有效厚度值";
      return;
    }
    panel.dirty=true;schedule();
  }

  function decode(panel) {
    if(panel.busy||!panel.pending)return;
    const data=panel.pending;panel.pending=null;panel.busy=true;
    const ownEpoch=epoch,ownSession=session,image=new Image();
    image.onload=()=>{
      if(ownEpoch!==epoch||ownSession!==session)return;
      panel.busy=false;
      if(image.naturalWidth!==data.width||image.naturalHeight!==data.height){panel.errors++;return;}
      panel.canvas.width=data.width;panel.canvas.height=data.height;
      panel.canvas.getContext("2d").drawImage(image,0,0);panel.empty.hidden=true;panel.displayed++;
      panel.values.textContent=data.source_width+"×"+data.source_height+" · "+data.source_encoding+(data.depth_visualization?" · 深度伪彩预览":"");
      decode(panel);
    };
    image.onerror=()=>{if(ownEpoch!==epoch)return;panel.busy=false;panel.errors++;panel.empty.hidden=false;panel.empty.textContent="图像解码失败";decode(panel);};
    if(typeof data.jpeg!=="string"||data.jpeg.length>800000){panel.busy=false;panel.errors++;return;}
    image.src="data:image/jpeg;base64,"+data.jpeg;
  }

  function chart(panel, rows, labels) {
    const canvas=panel.canvas,ctx=canvas.getContext("2d"),w=canvas.width,h=canvas.height;
    ctx.fillStyle="#050a10";ctx.fillRect(0,0,w,h);
    const all=rows.flat().filter(Number.isFinite);
    if(!all.length)return;
    const low=Math.min(...all),high=Math.max(...all),range=Math.max(1e-9,high-low);
    ctx.strokeStyle="#26384c";for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,i*h/4);ctx.lineTo(w,i*h/4);ctx.stroke();}
    rows.forEach((row,c)=>{ctx.strokeStyle=COLORS[c%COLORS.length];ctx.beginPath();let pen=false;row.forEach((v,i)=>{if(!Number.isFinite(v)){pen=false;return;}const x=i/Math.max(1,row.length-1)*(w-20)+10,y=h-20-(v-low)/range*(h-55);if(!pen)ctx.moveTo(x,y);else ctx.lineTo(x,y);pen=true;});ctx.stroke();ctx.fillStyle=COLORS[c%COLORS.length];ctx.fillText(labels[c]||"CH"+(c+1),10+c*76,15);});
    ctx.fillStyle="#adbdcd";ctx.fillText(fixed(high),w-90,32);ctx.fillText(fixed(low),w-90,h-6);
  }

  function cloud(panel) {
    const data=panel.data,xyz=data.xyz||[],ctx=panel.canvas.getContext("2d"),w=panel.canvas.width,h=panel.canvas.height;
    ctx.fillStyle="#050a10";ctx.fillRect(0,0,w,h);
    if(!xyz.length)return;
    let bound=1;
    const distances=[];for(let i=0;i<xyz.length;i+=3)distances.push(Math.hypot(xyz[i],xyz[i+1],xyz[i+2]));
    distances.sort((a,b)=>a-b);bound=Math.max(.1,distances[Math.floor(distances.length*.95)]||1);
    const scale=Math.min(w,h)*.42/bound*zoom,ca=Math.cos(angle),sa=Math.sin(angle),ct=Math.cos(tilt),st=Math.sin(tilt);
    ctx.strokeStyle="#34465c";ctx.beginPath();ctx.moveTo(w/2,0);ctx.lineTo(w/2,h);ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);ctx.stroke();
    for(let i=0;i<xyz.length;i+=3){const x=ca*xyz[i]-sa*xyz[i+1],y=sa*xyz[i]+ca*xyz[i+1],z=xyz[i+2];const px=w/2+x*scale,py=h/2-(ct*z-st*y)*scale;if(px<0||px>w||py<0||py>h)continue;ctx.fillStyle="hsl("+(180+Math.max(-1,Math.min(1,z/bound))*80)+",80%,65%)";ctx.fillRect(px,py,2,2);}
    panel.values.textContent="原始 "+text(data.point_count)+" 点 · 显示 "+text(data.displayed_points)+" 点 · 单位 m\n拖动旋转，滚轮缩放；显示单帧雷达点云。";
  }

  function schedule() {if(raf!=null)return;raf=requestAnimationFrame(()=>{raf=null;for(const panel of panels.values()){if(!panel.dirty||!panel.data)continue;panel.dirty=false;const data=panel.data;if(data.kind==="cloud")cloud(panel);else if(data.kind==="waveform"){chart(panel,data.channels||[],(data.channels||[]).map((_,i)=>"CH"+(i+1)));panel.values.textContent="每通道 "+text(data.sample_count)+" 个采样 · "+text(data.unit);}else if(data.kind==="means"){const rows=(data.values||[]).map((_,i)=>panel.history.map(v=>v[i]));chart(panel,rows,rows.map((_,i)=>"CH"+(i+1)));panel.values.textContent=(data.values||[]).map((v,i)=>"CH"+(i+1)+" "+fixed(v)).join(" · ")+" "+text(data.unit);}else if(data.kind==="imu"){const rows=Array.from({length:6},(_,i)=>panel.history.map(v=>v[i]));chart(panel,rows,["Ax","Ay","Az","Wx","Wy","Wz"]);panel.values.textContent="加速度 "+(data.acceleration||[]).map(fixed).join(" / ")+"\n角速度 "+(data.gyro||[]).map(fixed).join(" / ")+" · "+text(data.unit);}}});}

  function render(valueConnected) {
    if(typeof valueConnected==="boolean")connected=valueConnected;
    if(!api)return;
    const status=replay?.status||"idle",busy=["starting","playing","paused","stopping"].includes(status)||api.inFlight("replay.start");
    const collecting=["preflighting","starting","recording","stopping","finalizing"].includes(api.taskStatus());
    const allowed=action=>connected&&api.gate(action).allowed&&!api.inFlight(action);
    $("replay-start").disabled=!allowed("replay.start")||busy||collecting||!$("replay-run").value;
    $("quick-replay").disabled=$("replay-start").disabled;
    $("replay-pause").disabled=!allowed("replay.pause")||status!=="playing"||api.inFlight("replay.start");
    $("replay-resume").disabled=!allowed("replay.resume")||status!=="paused"||api.inFlight("replay.start");
    $("replay-stop").disabled=!allowed("replay.stop")||!["starting","playing","paused"].includes(status);
    $("replay-refresh").disabled=!allowed("history.list");$("replay-run").disabled=busy;$("replay-rate").disabled=busy;
    $("replay-status").textContent=LABELS[status]||status;$("replay-status").className="badge "+(status==="failed"?"bad":status==="playing"?"good":"muted");
    $("replay-message").textContent=failure||(collecting&&!busy?"正在采集，请先停止采集再回放。":replay?.message||"选择历史批次后点击一键回放。");
    $("replay-progress").style.width=Math.max(0,Math.min(100,replay?.progress_percent||0))+"%";
    $("replay-time").textContent=(replay?.position_seconds||0).toFixed(1)+" / "+(replay?.duration_seconds||0).toFixed(1)+" 秒 · "+(replay?.rate||1)+" 倍速";
  }

  function init(value) {
    api=value;
    const start=()=>{failure="";document.querySelector('nav button[data-view="replay"]').click();api.send("replay.start",{run_id:$("replay-run").value,rate:Number($("replay-rate").value)});};
    $("replay-start").onclick=start;$("quick-replay").onclick=start;
    $("replay-refresh").onclick=()=>api.send("history.list",{});
    for(const action of ["pause","resume","stop"])$("replay-"+action).onclick=()=>{failure="";api.send("replay."+action,{});};
    $("replay-run").onchange=()=>render();
  }

  function snapshot() {return {state:replay,session_id:session,panels:Object.fromEntries([...panels].map(([k,p])=>[k,{received:p.received,displayed:p.displayed,errors:p.errors,count:p.spec.count,last_stamp_ns:p.latest?.stamp_ns}]))};}
  window.TankReplay=Object.freeze({init,receive,render,snapshot});
})();
