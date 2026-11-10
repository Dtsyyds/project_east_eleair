/* Independent same-origin SLAM client. No chassis/inspection API access. */
"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const activeStates = new Set(["starting", "running", "stopping", "saving"]);
  const labels = {idle:"空闲", starting:"初始化中 · 请保持静止", running:"运行中", stopping:"正在停止", saving:"正在保存地图", stopped:"已停止", failed:"任务失败"};
  const saves = {success:"保存成功", disabled:"未启用保存", no_valid_map:"无有效地图", failed:"保存失败", unconfirmed:"保存未确认"};
  let socket, known=false, state=null, startIntent=null, stopPending=false, embeddedActive=true;
  let lastStatus=0, lastConfirmed=null, reconnectTimer, connectionTimer, attempts=0, preflightBusy=false;
  const healthLabels={valid:"有效",not_ready:"尚未就绪",tracking_invalid:"跟踪无效",odom_stale:"里程计过期",tracking_stale:"跟踪观测过期",odom_clock_invalid:"里程计时钟异常",unknown:"未知"};
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
  function checks(element, items) {
    element.replaceChildren();
    for(const item of items || []) {
      const li=document.createElement("li"), badge=document.createElement("span");
      badge.className=`badge ${item.status==="pass"?"good":item.status==="fail"?"bad":"warn"}`;
      badge.textContent=item.status==="pass"?"通过":item.status==="fail"?"未满足":"未知";
      li.append(badge,document.createTextNode(item.label+(item.suggestion?` · ${item.suggestion}`:"")));
      if(item.detail){const detail=document.createElement("details"),summary=document.createElement("summary"),text=document.createElement("div");summary.textContent="技术详情";text.textContent=item.detail;detail.append(summary,text);li.append(detail);}
      element.append(li);
    }
  }
  function savePath() {
    const mode=$("mode").value, root=state?.output_roots?.[mode];
    $("save-path").textContent=root || "等待后台配置";
    $("save-path-hint").textContent=(!known?"连接中断，以上为最后确认的目录。 ":"")+(!$("save").checked?"当前未启用保存地图。":"每次任务创建独立子目录，避免覆盖已有地图。");
    if(state?.output_directory)$("save-path-hint").textContent+=` ${activeStates.has(state.state)?"本次":"上次"}任务目录：${state.output_directory}`;
  }
  function render() {
    savePath();
    renderPosition();
    const busy=known && activeStates.has(state?.state);
    const stopping=stopPending || ["stopping","saving"].includes(state?.state);
    $("configuration").disabled=!known || busy || !!startIntent;
    $("preflight").disabled=!known || busy || !!startIntent || preflightBusy;
    $("map").disabled=$("mode").value!=="relocation";
    $("toggle").disabled=!known || stopping;
    $("toggle").className=busy || startIntent ? "stop":"start";
    $("toggle").textContent=!known?"状态未知":stopping?"正在停止…":busy||startIntent?"停止":"启动";
    $("state").textContent=known?(labels[state?.state]||"未知"):"连接中断 · 任务状态未知";
    const tone=!known?"warn":state?.state==="failed"?"bad":state?.state==="running"?"good":busy?"warn":"info";
    $("state").className=`badge ${tone}`;
    $("session").textContent=state?.session_id?`会话 ${state.session_id}`:"尚无会话";
    $("result").textContent=state?.save_result?`${saves[state.save_result]}${state.maps?.length?" · "+state.maps.join(" · "):""}`:"";
    const health=known?state?.health:null;
    $("health").textContent=`定位：${healthLabels[health?.state]||"未知"}`;
    $("health").className=`badge ${health?.state==="valid"?"good":"warn"}`;
    $("save-status").textContent=!known?"保存：未知":`保存：${saves[state?.save_result]||(state?.session_id?(state.save?"等待结果":"未启用保存"):"尚无会话")}`;
    $("save-status").className=`badge ${known&&state?.save_result==="success"?"good":known&&state?.save_result==="failed"?"bad":"info"}`;
    $("last-confirmed").textContent=lastConfirmed?`最后确认：${lastConfirmed.toLocaleTimeString()}${known?"":" · 信息已过期，保留上次会话标识"}`:"尚未确认后台状态";
    $("input-health").textContent=known?(health?.inputs||[]).map(i=>`${i.label}：${i.status==="pass"?"新鲜":i.status==="fail"?"过期":"未知"}${i.age==null?"":` ${i.age.toFixed(1)}s`}`).join(" · "):"输入健康未知";
    const init=known&&state?.state==="starting"?state.initialization:null;
    $("initialization").hidden=!init;
    if(init){$("initialization-time").textContent=`初始化已等待 ${init.elapsed.toFixed(1)}s / 超时上限 ${init.timeout}s · 请保持静止`;checks($("initialization-checks"),init.checks);}
  }
  function renderPosition() {
    const position=state?.position, running=["starting","running"].includes(state?.state);
    const show=(prefix,values)=>["x","y","z"].forEach((axis,i)=>{
      const value=values?.[i];
      $(prefix+"-"+axis).textContent=Number.isFinite(value)?(Math.abs(value)<.0005?0:value).toFixed(3):"—";
    });
    show("position",position?.current?.xyz);show("initial",position?.initial?.xyz);
    const valid=known&&running&&position?.fresh&&state?.health?.state==="valid";
    const badge=$("position-status");
    badge.className=`badge ${valid?"good":"warn"}`;
    badge.textContent=!known?"连接中断 · 状态未知":!state?.session_id?"等待任务启动":!running?(["stopping","saving"].includes(state?.state)?"正在停止 · 最后位置":"任务已停止 · 最后位置"):!position?.current?"等待定位":!position.fresh?"数据过期 · 最后位置":state?.health?.state!=="valid"?"定位未就绪 · 最近位置":"实时定位";
    $("position-frame").textContent=`坐标系：${position?.frame_id||"等待确认"} · 单位 m${known&&running&&position?.age!=null?` · 数据年龄 ${position.age.toFixed(1)}s`:""}`;
    $("initial-source").textContent=!state?.session_id?"等待任务启动":state.mode==="mapping"?"直接建图原点 · 本次任务固定":position?.initial?"GICP 配准初值（IMU 位置）· 本次任务固定":"等待本次 GICP 配准结果";
  }
  async function stop(sid) {
    if(stopPending)return;
    stopPending=true;render();
    try {accept(await request("stop",{},sid));}
    catch(e){error(e);}
    finally {stopPending=false;render();}
  }
  function accept(next) {
    if(!next || !Object.hasOwn(labels,next.state) || !("session_id" in next))return;
    if(state?.session_id!==next.session_id){cursor=0;historyCursor=0;logGeneration++;$("logs").textContent="";ansi.reset();cloud.clear();}
    state=next;known=true;
    lastStatus=performance.now();lastConfirmed=new Date();attempts=0;clearTimeout(connectionTimer);
    $("connection").textContent="已连接 · 状态已同步";$("connection").className="badge good";parentNotice(true);
    if(next.backend_version)$("versions").textContent=`页面资源 20261111-position · 后台 ${next.backend_version}`;
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
  const options=()=>({mode:$("mode").value,map:$("map").value,save:$("save").checked});
  $("configuration").addEventListener("change",()=>{savePath();$("preflight-summary").textContent="配置已变更，之前检查结果仅供参考；请重新检查。";});
  $("preflight").onclick=async()=>{
    if(!known||activeStates.has(state?.state)||preflightBusy)return;
    const params=options();preflightBusy=true;render();$("preflight-summary").textContent="正在检查启动条件…";
    try {const result=await request("preflight",params);checks($("preflight-results"),result.checks||result.errors?.map(detail=>({label:"启动检查",status:"fail",detail})));
      $("preflight-summary").textContent=`${result.ok?"检查通过":"检查未通过"} · ${new Date(result.checked_at*1000).toLocaleTimeString()} · ${JSON.stringify(params)!==JSON.stringify(options())?"配置已变化，请重新检查":"启动时仍会重新检查"}`;
    }catch(e){$("preflight-summary").textContent=`检查未完成：${e.message}`;}finally{preflightBusy=false;render();}
  };
  $("toggle").onclick=async()=>{
    if(!known)return;
    if(startIntent){startIntent.cancelled=true;$("state").textContent="已请求取消，等待本次会话标识…";return;}
    if(activeStates.has(state?.state)){void stop(state.session_id);return;}
    const intent={id:uid(),cancelled:false};startIntent=intent;$("error").textContent="";render();
    try {accept(await request("start",options(),null,intent.id));}
    catch(e){error(e);if(known && e.confirmed)startIntent=null;else {known=false;request("status",{start_request_id:startIntent?.id}).then(accept).catch(error);}}
    render();
  };
  function invalidate(message) {
    known=false;
    if(startIntent)startIntent.cancelled=false; // Reconnect never replays a control operation.
    $("connection").textContent=message;$("connection").className="badge warn";parentNotice(false);render();
  }
  function connect() {
    clearTimeout(reconnectTimer);clearTimeout(connectionTimer);
    invalidate("正在连接 · 任务状态未知");
    const ws=new WebSocket(`${location.protocol==="https:"?"wss:":"ws:"}//${location.host}/ws`);socket=ws;
    connectionTimer=setTimeout(()=>{if(socket!==ws)return;invalidate("连接超时 · 请检查服务及网络");ws.close();},8000);
    ws.onopen=async()=>{
      if(socket!==ws)return;
      clearTimeout(connectionTimer);
      $("connection").textContent="连接已建立 · 等待业务状态";
      connectionTimer=setTimeout(()=>{if(socket!==ws)return;invalidate("后台状态等待超时");ws.close();},8000);
      try {const result=await request("status",{start_request_id:startIntent?.id});if(socket!==ws)return;accept(result);await maps();}catch(e){error(e);}
    };
    ws.onmessage=event=>{
      if(socket!==ws)return;
      let m;try{m=JSON.parse(event.data);}catch(_){return;}
      if(m.type==="response"){
        const p=pending.get(m.request_id);if(!p)return;pending.delete(m.request_id);clearTimeout(p.timer);
        if(m.ok)p.resolve(m.result);else {const e=Error(m.error);e.confirmed=true;p.reject(e);}
      }else if(m.type==="status")accept(m);
      else if(m.type==="cloud" && m.session_id===state?.session_id && activeStates.has(state?.state))cloud.receive(m);
    };
    ws.onclose=()=>{
      if(socket!==ws)return;
      clearTimeout(connectionTimer);
      const delay=Math.min(15000,1000*2**Math.min(attempts++,4))*(.8+Math.random()*.2);
      const reason=$("connection").textContent;
      invalidate(`${reason.includes("超时")||reason.includes("过期")?reason:"连接断开"} · ${(delay/1000).toFixed(1)}s 后重连`);
      for(const p of pending.values()){clearTimeout(p.timer);p.reject(Error("连接断开，后台任务不会自动停止"));}pending.clear();
      reconnectTimer=setTimeout(connect,delay);
    };
    ws.onerror=()=>ws.close();
  }
  $("retry").onclick=()=>{
    attempts=0;clearTimeout(reconnectTimer);
    if(socket && socket.readyState!==WebSocket.CLOSED){invalidate("手动重新连接");socket.close();}
    else connect();
  };
  setInterval(()=>{
    if(known && performance.now()-lastStatus>5000){invalidate("状态已过期 · 任务状态未知");socket?.close();}
  },500);
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
      this.canvas=$("cloud");this.gl=null;this.failure=null;this.geometryDirty=true;this.pointCount=0;this.frames=[];this.key=null;this.last=null;this.yaw=.5;this.pitch=.65;this.zoom=1;this.dirty=true;
      this.canvas.addEventListener("webglcontextlost",e=>{e.preventDefault();this.fail(Error("WebGL 上下文丢失，等待恢复"));});
      this.canvas.addEventListener("webglcontextrestored",()=>this.initialize());
      this.initialize();
      let drag=null;this.canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];this.canvas.setPointerCapture(e.pointerId);};
      this.canvas.onpointermove=e=>{if(!drag)return;this.yaw+=(e.clientX-drag[0])*.01;this.pitch+=(e.clientY-drag[1])*.01;drag=[e.clientX,e.clientY];this.dirty=true;};
      this.canvas.onpointerup=this.canvas.onpointercancel=()=>{drag=null;};
      this.canvas.addEventListener("wheel",e=>{e.preventDefault();this.zoom=Math.max(.05,Math.min(100,this.zoom*Math.exp(-e.deltaY*.001)));this.dirty=true;},{passive:false});
      const draw=()=>{if(embeddedActive&&!document.hidden&&this.dirty&&this.gl){try{this.draw();}catch(e){this.fail(e);}}requestAnimationFrame(draw);};requestAnimationFrame(draw);
      new ResizeObserver(()=>{this.dirty=true;}).observe(this.canvas);
    }
    fail(e){this.failure=`点云预览不可用：${e.message||e}；状态、日志及任务控制仍可使用`;this.gl=null;$("cloud-info").textContent=this.failure;}
    initialize(){
      try {
      this.gl=this.canvas.getContext("webgl");
      if(!this.gl)throw Error("浏览器不支持 WebGL");
      const gl=this.gl;
      const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
      this.program=gl.createProgram();
      gl.attachShader(this.program,shader(gl.VERTEX_SHADER,"attribute vec3 p; uniform vec3 center; uniform vec4 view; uniform float aspect; varying float h; void main(){vec3 v=(p-center)*view.w;float x=cos(view.x)*v.x-sin(view.x)*v.y;float y=sin(view.x)*v.x+cos(view.x)*v.y;float z=sin(view.y)*y+cos(view.y)*v.z;float vertical=cos(view.y)*y-sin(view.y)*v.z;gl_Position=vec4(x/aspect,vertical,z*.15,1.);gl_PointSize=2.;h=clamp(v.z*.5+.5,0.,1.);}"));
      gl.attachShader(this.program,shader(gl.FRAGMENT_SHADER,"precision mediump float; varying float h; void main(){gl_FragColor=vec4(.15+h*.65,.65+h*.25,1.-h*.6,1.);}"));
      gl.linkProgram(this.program);if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(this.program));this.buffer=gl.createBuffer();
      this.failure=null;this.geometryDirty=true;this.dirty=true;this.info();
      }catch(e){this.fail(e);}
    }
    clear(){this.frames=[];this.key=null;this.last=null;this.geometryDirty=true;this.dirty=true;}
    trim(){const n=Math.max(1,Math.min(100,Math.floor(Number($("frames").value)||30)));$("frames").value=n;this.frames=this.frames.slice(-n);this.geometryDirty=true;this.dirty=true;}
    receive(m){if(m.topic!==$("topic").value)return;const key=`${m.session_id}|${m.topic}|${m.frame_id}`;if(key!==this.key){this.clear();this.key=key;}this.last=m;this.lastArrival=performance.now();if(m.error){this.dirty=true;return;}this.frames.push(m.points.slice(0,2000));this.trim();}
    info(){if(!this.gl)return;const m=this.last;if(!m){$("cloud-info").textContent=`等待 ${$("topic").value}`;return;}const age=(performance.now()-this.lastArrival)/1000+(m.age||0);$("cloud-info").textContent=`${m.topic} · frame_id: ${m.frame_id} · 帧时间 ${m.stamp.toFixed(3)} · ${m.points.length}/${m.source_points} 点 · 显示 ${this.frames.reduce((n,f)=>n+f.length,0)} 点 · ${age>2?"数据过期":"新鲜"} ${age.toFixed(1)}s${m.error?" · 解析异常："+m.error:!m.points.length?" · 空帧":""}`;}
    draw(){
      if(!this.gl)return;this.dirty=false;const gl=this.gl,c=this.canvas;const width=Math.round(c.clientWidth*Math.min(devicePixelRatio,2)),height=Math.round(c.clientHeight*Math.min(devicePixelRatio,2));if(c.width!==width)c.width=width;if(c.height!==height)c.height=height;gl.viewport(0,0,c.width,c.height);gl.clearColor(8/255,15/255,24/255,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
      gl.useProgram(this.program);gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);
      if(this.geometryDirty){
        this.pointCount=this.frames.reduce((n,f)=>n+f.length,0);
        const values=new Float32Array(this.pointCount*3),min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
        let offset=0;
        for(const frame of this.frames)for(const point of frame)for(let k=0;k<3;k++){
          const value=point[k];values[offset++]=value;min[k]=Math.min(min[k],value);max[k]=Math.max(max[k],value);
        }
        this.center=min.map((x,i)=>(x+max[i])/2);this.radius=Math.max(1,...min.map((x,i)=>max[i]-x));
        gl.bufferData(gl.ARRAY_BUFFER,values,gl.DYNAMIC_DRAW);this.geometryDirty=false;
      }
      if(!this.pointCount)return;
      const pos=gl.getAttribLocation(this.program,"p");gl.enableVertexAttribArray(pos);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,0,0);gl.uniform3fv(gl.getUniformLocation(this.program,"center"),this.center);gl.uniform4f(gl.getUniformLocation(this.program,"view"),this.yaw,this.pitch,0,1.6*this.zoom/this.radius);gl.uniform1f(gl.getUniformLocation(this.program,"aspect"),c.width/c.height);gl.enable(gl.DEPTH_TEST);gl.drawArrays(gl.POINTS,0,this.pointCount);
    }
  }
  const cloud=new PointView();
  const cloudPanel=$("cloud-panel"),fullscreenButton=$("cloud-fullscreen");
  fullscreenButton.disabled=!document.fullscreenEnabled;
  if(!document.fullscreenEnabled)fullscreenButton.title="当前浏览器或嵌入页面不允许全屏";
  fullscreenButton.onclick=async()=>{
    $("fullscreen-status").textContent="";
    try {
      if(document.fullscreenElement===cloudPanel)await document.exitFullscreen();
      else await cloudPanel.requestFullscreen();
    }catch(e){$("fullscreen-status").textContent=`无法进入全屏：${e.message}`;}
  };
  document.addEventListener("fullscreenchange",()=>{
    const full=document.fullscreenElement===cloudPanel;
    fullscreenButton.textContent=full?"退出全屏":"全屏";fullscreenButton.setAttribute("aria-pressed",String(full));cloud.dirty=true;
  });
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
  $("page-status").textContent="页面资源已加载";$("page-status").className="badge good";
  connect();
})();
