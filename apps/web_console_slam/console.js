/* Independent same-origin SLAM client. No chassis/inspection API access. */
"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const activeStates = new Set(["starting", "running", "stopping", "saving"]);
  const labels = {idle:"空闲", starting:"初始化中 · 请保持静止", running:"运行中", stopping:"正在停止", saving:"正在保存地图", stopped:"已停止", failed:"任务失败"};
  const saves = {success:"保存成功", disabled:"未启用保存", no_valid_map:"无有效地图", failed:"保存失败", unconfirmed:"保存未确认"};
  let socket, known=false, state=null, startIntent=null, stopPending=false, embeddedActive=true;
  let cursor=0, historyCursor=0, historyMode=false, logBusy=false, logGeneration=0, following=true;
  const pending=new Map();
  const uid=()=>globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(16).slice(2)}`;
  function request(op, params={}, sessionId=null, id=uid()) {
    return new Promise((resolve,reject)=>{
      if(socket?.readyState!==WebSocket.OPEN){reject(Error("后台未连接"));return;}
      const timer=setTimeout(()=>{pending.delete(id);reject(Error("请求超时，结果待状态查询确认"));},30000);
      pending.set(id,{resolve,reject,timer});
      socket.send(JSON.stringify({op,params,session_id:sessionId,request_id:id}));
    });
  }
  function error(e){$("error").textContent=e.message||String(e);}
  function parentNotice(connected) {
    if(parent!==window) parent.postMessage({type:"slam-ready",connected}, `${location.protocol}//${location.hostname}:8080`);
  }
  function render() {
    const busy=known && activeStates.has(state?.state);
    const stopping=stopPending || ["stopping","saving"].includes(state?.state);
    $("configuration").disabled=!known || busy || !!startIntent;
    $("map").disabled=$("mode").value!=="relocation";
    $("toggle").disabled=!known || stopping;
    $("toggle").className=busy || startIntent ? "stop":"start";
    $("toggle").textContent=!known?"状态未知":stopping?"正在停止…":busy||startIntent?"停止":"启动";
    $("state").textContent=known?(labels[state?.state]||"未知"):"连接中断 · 任务状态未知";
    const tone=!known?"warn":state?.state==="failed"?"bad":state?.state==="running"?"good":busy?"warn":"info";
    $("state").className=`badge ${tone}`;
    $("session").textContent=state?.session_id?`会话 ${state.session_id}`:"尚无会话";
    $("result").textContent=state?.save_result?`${saves[state.save_result]}${state.maps?.length?" · "+state.maps.join(" · "):""}`:"";
  }
  async function stop(sid) {
    if(stopPending)return;
    stopPending=true;render();
    try {accept(await request("stop",{},sid));}
    catch(e){error(e);}
    finally {stopPending=false;render();}
  }
  function accept(next) {
    if(!next || !("state" in next))return;
    if(state?.session_id!==next.session_id){cursor=0;historyCursor=0;logGeneration++;$("logs").textContent="";ansi.reset();cloud.clear();}
    state=next;known=true;
    if(next.start_resolution==="not_accepted")startIntent=null;
    if(activeStates.has(state.state)) {
      $("mode").value=state.mode;
      $("save").checked=state.save;
      if(state.map){const m=[...$("map").options].find(x=>x.dataset.path===state.map);if(m)$("map").value=m.value;$("map-path").textContent=state.map;}
    }
    if(state.error)error(state.error);
    if(startIntent && state.start_request_id===startIntent.id){
      const cancelled=startIntent.cancelled;startIntent=null;
      if(cancelled && activeStates.has(state.state))void stop(state.session_id);
    }
    render();
  }
  async function maps() {
    const list=await request("maps.list");
    const selected=$("map").value;$("map").replaceChildren();
    for(const map of list){const option=document.createElement("option");option.value=map.name;option.textContent=map.name;option.dataset.path=map.path;$("map").append(option);}
    if(!list.length){const o=document.createElement("option");o.value="";o.textContent="无可用先验地图";$("map").append(o);}
    const chosen=activeStates.has(state?.state)?list.find(m=>m.path===state.map)?.name:selected;
    if(list.some(m=>m.name===chosen))$("map").value=chosen;
    path();
  }
  function path(){ $("map").disabled=$("mode").value!=="relocation";$("map-path").textContent=$("mode").value==="mapping"?"直接建图无需先验地图":($("map").selectedOptions[0]?.dataset.path||"请将 PCD 放入配置的先验目录后刷新"); }
  $("mode").onchange=path;$("map").onchange=path;$("refresh-maps").onclick=()=>maps().catch(error);
  $("toggle").onclick=async()=>{
    if(startIntent){startIntent.cancelled=true;$("state").textContent="已请求取消，等待本次会话标识…";return;}
    if(activeStates.has(state?.state)){void stop(state.session_id);return;}
    const intent={id:uid(),cancelled:false};startIntent=intent;$("error").textContent="";render();
    try {accept(await request("start",{mode:$("mode").value,map:$("map").value,save:$("save").checked},null,intent.id));}
    catch(e){error(e);if(known && e.confirmed)startIntent=null;else {known=false;request("status",{start_request_id:startIntent?.id}).then(accept).catch(error);}}
    render();
  };
  function connect() {
    $("connection").textContent="连接中 · 状态未知";
    $("connection").className="badge warn";
    socket=new WebSocket(`${location.protocol==="https:"?"wss:":"ws:"}//${location.host}/ws`);
    socket.onopen=async()=>{
      $("connection").textContent="已连接";$("connection").className="badge good";parentNotice(true);
      try {accept(await request("status",{start_request_id:startIntent?.id}));await maps();}catch(e){error(e);}
    };
    socket.onmessage=event=>{
      let m;try{m=JSON.parse(event.data);}catch(_){return;}
      if(m.type==="response"){
        const p=pending.get(m.request_id);if(!p)return;pending.delete(m.request_id);clearTimeout(p.timer);
        if(m.ok)p.resolve(m.result);else {const e=Error(m.error);e.confirmed=true;p.reject(e);}
      }else if(m.type==="status")accept(m);
      else if(m.type==="cloud" && m.session_id===state?.session_id && activeStates.has(state?.state))cloud.receive(m);
    };
    socket.onclose=()=>{
      known=false;$("connection").textContent="未连接 · 自动重连中";$("connection").className="badge warn";parentNotice(false);render();
      for(const p of pending.values()){clearTimeout(p.timer);p.reject(Error("连接断开，后台任务不会自动停止"));}pending.clear();
      setTimeout(connect,1500);
    };
    socket.onerror=()=>socket.close();
  }
  // Stateful ANSI removal handles CSI/OSC escape sequences split across reads.
  const ansi={mode:"text",reset(){this.mode="text";},clean(text){let out="";for(const c of text){if(this.mode==="text"){if(c==="\x1b")this.mode="esc";else if(c==="\n"||c==="\t"||c.charCodeAt(0)>=32)out+=c;}else if(this.mode==="esc")this.mode=c==="["?"csi":c==="]"?"osc":"text";else if(this.mode==="csi"){if(c>="@"&&c<="~")this.mode="text";}else if(this.mode==="osc"){if(c==="\x07")this.mode="text";else if(c==="\x1b")this.mode="osc-end";}else if(this.mode==="osc-end")this.mode=c==="\\"?"text":"osc";}return out;}};
  async function logs(history=false) {
    if(!known || !state?.session_id || logBusy)return;
    logBusy=true;const generation=logGeneration;
    try {
      const m=await request("logs.read",{cursor:history?historyCursor:cursor},state.session_id);
      if(generation!==logGeneration)return;
      if(history)historyCursor=m.next_cursor;else cursor=m.next_cursor;
      const el=$("logs");el.textContent=(el.textContent+ansi.clean(m.text)).slice(-262144);
      if(following)el.scrollTop=el.scrollHeight;
      $("log-position").textContent=`${history?"历史":"实时"} · 字节 ${m.cursor}–${m.next_cursor} / ${m.size} · 页面显示缓冲 256 KiB；完整记录在后台`;
      $("next-history").disabled=!historyMode || m.next_cursor>=m.size;
    }catch(e){error(e);}finally{logBusy=false;}
  }
  $("follow").onclick=()=>{following=!following;$("follow").textContent=following?"暂停跟随":"跟随最新";};
  $("clear-log").onclick=()=>{$("logs").textContent="";};
  $("history").onclick=()=>{historyMode=true;historyCursor=0;logGeneration++;ansi.reset();$("logs").textContent="";if(!logBusy)void logs(true);else setTimeout(()=>logs(true),400);};
  $("next-history").onclick=()=>logs(true);
  $("live").onclick=()=>{historyMode=false;logGeneration++;ansi.reset();$("next-history").disabled=true;};
  setInterval(()=>{if(!historyMode)void logs();},300);

  class PointView {
    constructor(){
      this.canvas=$("cloud");this.gl=this.canvas.getContext("webgl");this.frames=[];this.key=null;this.last=null;this.yaw=.5;this.pitch=.65;this.zoom=1;this.dirty=true;
      if(!this.gl){$("cloud-info").textContent="浏览器不支持 WebGL，点云预览不可用";return;}
      const gl=this.gl;
      const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
      this.program=gl.createProgram();
      gl.attachShader(this.program,shader(gl.VERTEX_SHADER,"attribute vec3 p; uniform vec3 center; uniform vec4 view; uniform float aspect; varying float h; void main(){vec3 v=(p-center)*view.w;float x=cos(view.x)*v.x-sin(view.x)*v.y;float y=sin(view.x)*v.x+cos(view.x)*v.y;float z=sin(view.y)*y+cos(view.y)*v.z;float vertical=cos(view.y)*y-sin(view.y)*v.z;gl_Position=vec4(x/aspect,vertical,z*.15,1.);gl_PointSize=2.;h=clamp(v.z*.5+.5,0.,1.);}"));
      gl.attachShader(this.program,shader(gl.FRAGMENT_SHADER,"precision mediump float; varying float h; void main(){gl_FragColor=vec4(.15+h*.65,.65+h*.25,1.-h*.6,1.);}"));
      gl.linkProgram(this.program);if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(this.program));this.buffer=gl.createBuffer();
      let drag=null;this.canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];this.canvas.setPointerCapture(e.pointerId);};
      this.canvas.onpointermove=e=>{if(!drag)return;this.yaw+=(e.clientX-drag[0])*.01;this.pitch+=(e.clientY-drag[1])*.01;drag=[e.clientX,e.clientY];this.dirty=true;};
      this.canvas.onpointerup=this.canvas.onpointercancel=()=>{drag=null;};
      this.canvas.addEventListener("wheel",e=>{e.preventDefault();this.zoom=Math.max(.05,Math.min(100,this.zoom*Math.exp(-e.deltaY*.001)));this.dirty=true;},{passive:false});
      const draw=()=>{if(embeddedActive&&!document.hidden&&this.dirty)this.draw();requestAnimationFrame(draw);};requestAnimationFrame(draw);
      new ResizeObserver(()=>{this.dirty=true;}).observe(this.canvas);
    }
    clear(){this.frames=[];this.key=null;this.last=null;this.dirty=true;}
    trim(){const n=Math.max(1,Math.min(100,Number($("frames").value)||30));$("frames").value=n;this.frames=this.frames.slice(-n);this.dirty=true;}
    receive(m){if(m.topic!==$("topic").value)return;const key=`${m.session_id}|${m.topic}|${m.frame_id}`;if(key!==this.key){this.clear();this.key=key;}this.last=m;this.lastArrival=Date.now();if(m.error){this.dirty=true;return;}this.frames.push(m.points.slice(0,2000));this.trim();}
    info(){if(!this.gl)return;const m=this.last;if(!m){$("cloud-info").textContent=`等待 ${$("topic").value}`;return;}const age=(Date.now()-this.lastArrival)/1000+(m.age||0);$("cloud-info").textContent=`${m.topic} · frame_id: ${m.frame_id} · 帧时间 ${m.stamp.toFixed(3)} · ${m.points.length}/${m.source_points} 点 · 显示 ${this.frames.reduce((n,f)=>n+f.length,0)} 点 · ${age>2?"数据过期":"新鲜"} ${age.toFixed(1)}s${m.error?" · 解析异常："+m.error:!m.points.length?" · 空帧":""}`;}
    draw(){
      if(!this.gl)return;this.dirty=false;const gl=this.gl,c=this.canvas;c.width=Math.round(c.clientWidth*Math.min(devicePixelRatio,2));c.height=Math.round(c.clientHeight*Math.min(devicePixelRatio,2));gl.viewport(0,0,c.width,c.height);gl.clearColor(8/255,15/255,24/255,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
      const values=this.frames.flat(2);if(!values.length)return;
      const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];for(let i=0;i<values.length;i++){const k=i%3;min[k]=Math.min(min[k],values[i]);max[k]=Math.max(max[k],values[i]);}
      const center=min.map((x,i)=>(x+max[i])/2),radius=Math.max(1,...min.map((x,i)=>max[i]-x));
      gl.useProgram(this.program);gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(values),gl.DYNAMIC_DRAW);const pos=gl.getAttribLocation(this.program,"p");gl.enableVertexAttribArray(pos);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,0,0);gl.uniform3fv(gl.getUniformLocation(this.program,"center"),center);gl.uniform4f(gl.getUniformLocation(this.program,"view"),this.yaw,this.pitch,0,1.6*this.zoom/radius);gl.uniform1f(gl.getUniformLocation(this.program,"aspect"),c.width/c.height);gl.enable(gl.DEPTH_TEST);gl.drawArrays(gl.POINTS,0,values.length/3);
    }
  }
  const cloud=new PointView();
  $("topic").onchange=()=>cloud.clear();$("frames").onchange=()=>cloud.trim();$("clear-cloud").onclick=()=>cloud.clear();
  $("top").onclick=()=>{cloud.yaw=0;cloud.pitch=0;cloud.dirty=true;};$("reset").onclick=()=>{cloud.yaw=.5;cloud.pitch=.65;cloud.zoom=1;cloud.dirty=true;};setInterval(()=>{cloud.info();parentNotice(known);},1000);
  let lastHeight=0;
  function reportHeight() {
    if(!document.documentElement.classList.contains("embedded") || !document.documentElement.clientWidth)return;
    const height=Math.ceil(document.querySelector("main").getBoundingClientRect().height);
    if(height===lastHeight)return;
    lastHeight=height;
    parent.postMessage({type:"slam-size",height},`${location.protocol}//${location.hostname}:8080`);
  }
  new ResizeObserver(reportHeight).observe(document.querySelector("main"));
  window.addEventListener("message",event=>{
    if(parent===window || event.source!==parent || event.origin!==`${location.protocol}//${location.hostname}:8080`)return;
    if(event.data?.type==="slam-page-active"){
      document.documentElement.classList.add("embedded");
      embeddedActive=!!event.data.active;cloud.dirty=true;
      requestAnimationFrame(reportHeight);
    }
  });
  document.addEventListener("visibilitychange",()=>{cloud.dirty=true;});
  connect();
})();
