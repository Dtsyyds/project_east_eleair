/* 现有底盘 JSON 协议的独立客户端；不把网页窗口身份当作 STM32 控制权。 */
"use strict";
(() => {
  const $ = id => document.getElementById(id), now = () => performance.now();
  const client = globalThis.crypto?.randomUUID?.() || `page-${Date.now()}-${Math.random()}`;
  let ws = null, generation = 0, sequence = 0, feedback = null, received = 0, pageActive = true;
  let motion = [0, 0], probes = [0, 0], pending = new Map();
  let desired = false, retryTimer = null, retryDelay = 1000;
  let resumeConnectionOnPageShow = false;
  let tuningLoaded = false, tuningBusy = false;
  // 曾确认接管后，缺少反馈不能当成交还；仅新鲜明确的释放状态解锁。
  let remoteLocked = false;
  // 嵌入时由主页显示弹窗，避免提示藏在未打开的底盘 iframe 中。
  const takeoverNotice=parent===window?window.createTakeoverNotice(document.body):null;
  let lastTakeoverSnapshot="";
  function updateTakeoverNotice() {
    const a=fresh()?feedback.authority:null, p=fresh()?feedback.probe:null;
    const snapshot={
      chassis:a?.fresh===true&&[0,1,2].includes(a.chassis_owner)?(a.chassis_owner===1?"remote":feedback?.remote_takeover?.active?"unknown":"released"):"unknown",
      probe:p?.valid===true&&[0,1,2,3,4].includes(p.owner)?(p.owner===1?"remote":"released"):"unknown",
    };
    if(feedback?.remote_takeover?.active===true||snapshot.chassis==="remote")remoteLocked=true;
    else if(snapshot.chassis==="released")remoteLocked=false;
    const key=JSON.stringify(snapshot);if(key===lastTakeoverSnapshot)return;
    lastTakeoverSnapshot=key;
    if(takeoverNotice)takeoverNotice.update(snapshot);
    else parent.postMessage({type:"robot-remote-state",snapshot},location.origin==="null"?"*":location.origin);
  }
  const tuningLabels = {cruise_speed_mps:"巡航速度 m/s",min_speed_mps:"最低速度 m/s",max_angular_radps:"最大角速度 rad/s",
    lateral_kp:"横向 Kp",lateral_ki:"横向 Ki",lateral_kd:"横向 Kd",heading_kp:"方向 Kp",
    image_timeout_s:"图像超时 s",receive_timeout_s:"接收超时 s",min_confidence:"最低置信度"};
  const fmt = (v, n=3) => Number.isFinite(v) ? v.toFixed(n) : "未提供";
  const fresh = () => ws?.readyState === WebSocket.OPEN && feedback && now() - received < 600;
  const manualGate = () => fresh() && !remoteLocked && feedback.control?.enabled === true;
  const probeGate = () => fresh() && !remoteLocked && feedback.probe?.control_enabled === true;
  const editable = e => e.target?.closest?.("input,select,textarea,[contenteditable=true]");
  function tell(text) { $("notice").textContent = text; }
  function linkState(label, connected=false) {
    $("connection").textContent=label;
    if (parent !== window) parent.postMessage({type:"robot-link-state",label,connected}, location.origin === "null" ? "*" : location.origin);
  }
  function send(action, parameters = {}, tracked = false) {
    if(remoteLocked&&!action.endsWith(".stop")){tell("遥控器接管中，网页控制已锁定");return false;}
    if (!ws || ws.readyState !== WebSocket.OPEN || ws.bufferedAmount > 32768) return false;
    const request_id = `${client}-${++sequence}`;
    const payload = {action, parameters, request_id, client_id:client, token:$("token").value};
    ws.send(JSON.stringify({header:{robot_id:"tank_inspection_robot", msg_type:"command"}, payload}));
    if (tracked) pending.set(request_id, {at:now(), action});
    return request_id;
  }
  // 只接受对应 request_id 的终态响应；连接丢失和超时不伪装成成功。
  function request(action, parameters={}) {
    return new Promise((resolve,reject)=>{
      const id=send(action,parameters,true);
      if(!id){reject(Error("底盘桥接未连接"));return;}
      Object.assign(pending.get(id),{resolve,reject});
    });
  }
  function rejectPending(reason) {
    for(const p of pending.values())p.reject?.(Error(reason));
    pending.clear();tuningLoaded=false;
  }
  function cancelStart() {
    if([...pending.values()].some(p=>p.action==="tracking.start")) {
      // 取消正在等待的启用请求：断开其所属连接，服务端撤销授权；重连不重放。
      connect();return true;
    }
    return false;
  }
  window.RobotTrackingControl=Object.freeze({
    snapshot:()=>fresh()?structuredClone(feedback):null, request, cancelStart,
  });
  function stopManual(disarm = false) {
    if (motion.some(Boolean)) send("chassis.stop", {});
    motion = [0, 0];
    if (disarm) $("manual").checked = false;
  }
  function stopProbes(disarm = false) {
    if (probes.some(Boolean)) send("probe.stop", {});
    probes = [0, 0];
    if (disarm) $("probe").checked = false;
  }
  function limits() {
    const speed = Number($("speed").value), turn = Number($("turn").value);
    const cap = feedback?.capabilities?.control?.limits;
    if (!Number.isFinite(speed) || !Number.isFinite(turn) || speed < 0 || turn < 0 ||
        speed > (cap?.vx ?? .1) || turn > (cap?.wz ?? .3)) return null;
    return [speed, turn];
  }
  function transmitManual() {
    const limit = limits();
    if (!pageActive || !$("manual").checked || !manualGate() || !limit) {stopManual(true); return;}
    send("chassis.move", {vx:motion[0] * limit[0], wz:motion[1] * limit[1]});
  }
  function transmitProbes() {
    const percent = Number($("duty").value);
    if (!pageActive || !$("probe").checked || !probeGate() || !Number.isFinite(percent) || percent < 1 || percent > 100) {stopProbes(true);return;}
    send("probe.move", {duty:probes.map(v => v * Math.round(percent * 10))});
  }
  function connect() {
    disconnect();
    desired = true;
    let target;
    try {target = new URL($("endpoint").value);if (!["ws:","wss:"].includes(target.protocol)) throw Error("使用 ws:// 或 wss:// 地址");}
    catch(error) {desired=false;tell(error.message);return;}
    const mine = ++generation;
    const socket = ws = new WebSocket(target.href);
    linkState("连接中");
    socket.onopen = () => {if(mine!==generation)return;retryDelay=1000;linkState("已连接",true);tell("连接成功；手动与循迹均需人工启用");};
    socket.onmessage = event => {
      if (mine !== generation || typeof event.data !== "string" || event.data.length > 1048576) return;
      let packet;try {packet = JSON.parse(event.data);}catch {tell("收到非 JSON 消息");return;}
      if(packet.header?.robot_id !== "tank_inspection_robot")return;
      if(packet.header.msg_type === "status_update") {feedback=packet.payload;received=now();render();}
      if(packet.header.msg_type === "response") {
        const p=packet.payload, waiting=pending.get(p.request_id);
        if(waiting&&p.action===waiting.action&&["completed","failed"].includes(p.status)){
          pending.delete(p.request_id);
          if(p.status==="completed")waiting.resolve?.(p);else waiting.reject?.(Error(p.error||p.reason||"请求失败"));
        }
        if(packet.payload.status === "failed") tell("请求未执行：" + (packet.payload.error || packet.payload.reason));
        else if(packet.payload.action?.startsWith("tracking.")) tell("循迹服务：" + (packet.payload.reason || packet.payload.status));
      }
    };
    socket.onerror = () => {if(mine===generation)tell("连接失败：检查统一入口及 RDK 底盘桥接进程");};
    socket.onclose = () => {if(mine!==generation)return;ws=null;feedback=null;rejectPending("连接断开，操作结果待确认");stopManual(true);stopProbes(true);linkState(desired?"等待重连":"已断开");render();
      if(desired){retryTimer=setTimeout(()=>{retryTimer=null;if(desired)connect();},retryDelay);retryDelay=Math.min(retryDelay*2,8000);}
    };
  }
  function disconnect() {
    desired=false;if(retryTimer!==null){clearTimeout(retryTimer);retryTimer=null;}
    stopManual(true);stopProbes(true);
    if(ws) {ws.close();ws=null;}
    ++generation;feedback=null;rejectPending("连接已关闭");linkState("未连接");render();
  }
  function list(node, entries) {
    node.replaceChildren(...entries.flatMap(([name,value]) => {const dt=document.createElement("dt"), dd=document.createElement("dd");dt.textContent=name;dd.textContent=String(value);return [dt,dd];}));
  }
  function render() {
    updateTakeoverNotice();
    const ok=fresh(), t=ok?feedback.tracking:null, c=ok?feedback.chassis:null, a=ok?feedback.authority:null;
    if(!manualGate())stopManual(true);
    if(!probeGate())stopProbes(true);
    $("manual").disabled=!manualGate();$("probe").disabled=!probeGate();
    $("remote-control-state").textContent=remoteLocked?"遥控器接管中：回中只停车，网页运动和调参已禁用。确认停稳并持续回中至少0.5秒后，将CH11从低拨到高申请交还；交还后仍须点击启动循迹。":"网页操作需人工启用；CH11交还不自动续跑，关闭提示也不会恢复运动。";
    ["speed","turn","duty"].forEach(id=>$(id).disabled=remoteLocked);
    Object.keys(tuningLabels).forEach(name=>$("tuning-"+name).disabled=remoteLocked||!tuningLoaded||tuningBusy);
    document.querySelectorAll("[data-move]:not(.blank)").forEach(b=>b.disabled=!manualGate()||!$("manual").checked);
    document.querySelectorAll("[data-probe]").forEach(b=>b.disabled=!probeGate()||!$("probe").checked);
    $("manual-reason").textContent=ok?(feedback.control?.reason||"就绪"):"反馈未收到或已过期";
    $("probe-reason").textContent=ok?(feedback.probe?.control_reason||"就绪"):"反馈未收到或已过期";
    $("tracking-reason").textContent=t?.fresh?`${t.enabled?"已授权":"未启用"} · ${t.reason}`:"循迹节点状态未收到或已过期";
    $("tracking-mode").textContent=t?.fresh?(t.dry_run?"预览模式：不会下发到底盘":"实车模式：点击启动后可能运动，请确认安全"):"启动系统不会自动启用运动";
    $("tracking-start").disabled=remoteLocked||!t?.fresh||!t.service_ready||t.enabled||pending.size>0||$("manual").checked||$("probe").checked;
    $("tracking-stop").disabled=!ok||!t?.service_ready;
    $("tuning-read").disabled=remoteLocked||!ok||!feedback?.tracking_tuning?.available||tuningBusy||pending.size>0;
    $("tuning-apply").disabled=$("tuning-read").disabled||!tuningLoaded||!t?.fresh||t.enabled;
    // motor_fault 是打包状态码：0x1010 可表示正常使能，不把非零值直接标成故障。
    const motorCode=Number.isInteger(c?.motor_fault)?"0x"+c.motor_fault.toString(16).padStart(4,"0"):"未知";
    const entries=[["车型",c?.drive_mode??"未知"],["串口",ok?(feedback.driver?.connected?"已连接":"未连接"):"未知"],["底盘控制权",remoteLocked?`遥控器接管中 · CH11 ${a?.fresh?(a.ch11_high?"高（不代表已交还）":"低"):"未知"}`:a?.fresh?`owner=${a.chassis_owner}，等待人工操作`:"未收到权限帧"],["实测线速度 m/s",fmt(c?.linear_velocity_mps)],["实测角速度 rad/s",fmt(c?.angular_velocity_rad_s)],["电机状态码 / 欠压",c?`${motorCode} / ${c.undervoltage??"未知"}`:"未知"]];
    $("summary").replaceChildren(...entries.map(([name,value])=>{const d=document.createElement("div"),s=document.createElement("strong");d.className="tile";d.textContent=name;s.textContent=String(value);d.append(s);return d;}));
    list($("tracking-values"), [["横向误差",fmt(t?.lateral_error_norm)],["方向误差 rad",fmt(t?.heading_error_rad)],["置信度",fmt(t?.confidence)],["图像年龄 s",fmt(t?.image_age_s)],["接收年龄 s",fmt(t?.receive_age_s)],["请求线速度 m/s",fmt(t?.command_linear_mps)],["请求角速度 rad/s",fmt(t?.command_angular_radps)],["原始角速度 rad/s",fmt(t?.raw_angular_radps)]]);
    $("probe-values").textContent="实际施加占空比（‰）："+(ok?JSON.stringify(feedback.probe?.applied_duty??null):"未收到");
    const rec = ok ? feedback.recording : null;
    $("recording-status").textContent=rec?`${rec.current_session?"本次":"最近历史/未开始"} · ${rec.session_id||"尚无会话"} · 清单状态 ${rec.status}`:"录制清单未收到或连接过期";
    list($("recording-modules"), Object.entries(rec?.modules||{}).map(([name,m])=>[m.title||name,`${m.status??"未知"} · 消息 ${m.message_count??"收尾后统计"}`]));
    $("raw").textContent=ok?JSON.stringify(feedback,null,2):"反馈未收到或已过期";
  }
  const endpointHost=location.hostname.includes(":")?`[${location.hostname}]`:(location.hostname||"127.0.0.1");
  $("endpoint").value=new URLSearchParams(location.search).get("ws") || `${location.protocol==="https:"?"wss:":"ws:"}//${endpointHost}:9001/ws/robot`;
  $("connect").onclick=connect;$("disconnect").onclick=disconnect;
  $("manual").onchange=()=>{if(!$("manual").checked)stopManual();render();};
  $("probe").onchange=()=>{if(!$("probe").checked)stopProbes();render();};
  $("manual-stop").onclick=()=>stopManual();$("probe-stop").onclick=()=>stopProbes();
  $("tracking-start").onclick=()=>{stopManual(true);stopProbes(true);send("tracking.start",{},true);render();};
  $("tracking-stop").onclick=()=>{parent.dispatchEvent(new Event("robot-explicit-stop"));cancelStart();send("tracking.stop",{},true);render();};
  for(const [name,label] of Object.entries(tuningLabels)){
    const holder=document.createElement("label"),input=document.createElement("input");
    holder.textContent=label;input.id="tuning-"+name;input.type="number";input.step="0.001";input.disabled=true;
    holder.append(input);$("tuning-fields").append(holder);
  }
  async function tune(apply) {
    tuningBusy=true;render();
    try {
      const values=Object.fromEntries(Object.keys(tuningLabels).map(name=>[name,Number($("tuning-"+name).value)]));
      if(apply&&Object.keys(tuningLabels).some(name=>$("tuning-"+name).value.trim()===""||!Number.isFinite(values[name])))throw Error("请输入完整的有限数值");
      const reply=await request(apply?"tracking.parameters.set":"tracking.parameters.get",apply?values:{});
      if(!reply.result?.values||!reply.result?.limits)throw Error("节点未返回有效参数，不能确认应用成功");
      for(const name of Object.keys(tuningLabels)){
        const input=$("tuning-"+name),value=reply.result.values[name],range=reply.result.limits[name];
        if(!Number.isFinite(value)||!Array.isArray(range))throw Error("参数回读不完整");
        input.value=value;input.min=range[0];input.max=range[1];input.disabled=false;
      }
      tuningLoaded=true;$("tuning-notice").textContent=apply?"参数已应用并回读确认；未启动循迹。重启后恢复 YAML。":"已读取节点当前参数；暂停循迹后可应用。";
    }catch(error){$("tuning-notice").textContent=error.message;}
    finally{tuningBusy=false;render();}
  }
  $("tuning-read").onclick=()=>tune(false);$("tuning-apply").onclick=()=>tune(true);
  function hold(button, begin, end) {
    button.onpointerdown=e=>{if(e.button!==0)return;e.preventDefault();button.setPointerCapture?.(e.pointerId);begin();};
    ["pointerup","pointercancel","lostpointercapture"].forEach(name=>button.addEventListener(name,end));
  }
  document.querySelectorAll("[data-move]:not(.blank)").forEach(b=>hold(b,()=>{motion=b.dataset.move.split(",").map(Number);transmitManual();},()=>stopManual()));
  document.querySelectorAll("[data-probe]").forEach(b=>hold(b,()=>{const [i,s]=b.dataset.probe.split(",").map(Number);probes[i]=s;transmitProbes();},()=>{probes=[0,0];send("probe.stop",{});}));
  // ROS 键位与 WASD；斜向键同时给前进速度和角速度。
  const movement={w:[1,0],i:[1,0],s:[-1,0],",":[-1,0],a:[0,1],j:[0,1],d:[0,-1],l:[0,-1],u:[1,1],o:[1,-1],m:[-1,-1],".":[-1,1]};
  document.addEventListener("keydown",e=>{if(editable(e)||!pageActive)return;const k=e.key.toLowerCase();if(k==="k"||k===" "){e.preventDefault();stopManual();return;}if(movement[k]&&$("manual").checked){e.preventDefault();motion=movement[k];transmitManual();}});
  // 松键必须停车，即使按住期间焦点已进入输入框；不能因编辑态过滤而留下运动。
  document.addEventListener("keyup",e=>{if(movement[e.key.toLowerCase()])stopManual();});
  document.addEventListener("focusin",e=>{if(editable(e)){stopManual();stopProbes();}});
  window.addEventListener("blur",()=>{stopManual(true);stopProbes(true);});
  document.addEventListener("visibilitychange",()=>{if(document.hidden){stopManual(true);stopProbes(true);}});
  window.addEventListener("message",e=>{
    if(e.source!==parent)return;
    if(e.data?.type==="robot-page-active"){pageActive=e.data.active===true;if(!pageActive){stopManual(true);stopProbes(true);}}
    if(e.data?.type==="robot-link-disconnect")disconnect();
    if(e.data?.type==="robot-link-connect"&&typeof e.data.endpoint==="string"){$("endpoint").value=e.data.endpoint;connect();}
  });
  window.addEventListener("pagehide",()=>{resumeConnectionOnPageShow=desired;disconnect();});
  window.addEventListener("pageshow",event=>{if(event.persisted&&resumeConnectionOnPageShow)connect();});
  setInterval(()=>{if(motion.some(Boolean))transmitManual();if(probes.some(Boolean))transmitProbes();for(const [id,p] of pending)if(now()-p.at>6000){pending.delete(id);p.reject?.(Error("请求响应超时，结果未知，请重新读取状态"));tell("请求响应超时，请核对循迹状态；不要把按钮请求当成运动成功");}render();},100);
  render();connect();
})();
