/* 页面加载即连接；相机与底盘按路径共用同一个对外端口。 */
"use strict";
(() => {
  const frame = document.getElementById("robot-frame");
  const section = document.getElementById("view-chassis");
  document.getElementById("workspace").appendChild(section);
  const params = new URLSearchParams(location.search);
  const host=location.hostname.includes(":")?`[${location.hostname}]`:(location.hostname||"127.0.0.1");
  const endpoint = params.get("robot_ws") || window.RobotConnectionDefaults?.robot || `${location.protocol === "https:" ? "wss:" : "ws:"}//${host}:9001/ws/robot`;
  frame.src = "robot/index.html?ws=" + encodeURIComponent(endpoint);
  const badge = document.createElement("span");
  badge.id = "robot-connection";badge.className = "badge warn";badge.textContent = "底盘连接中";
  document.querySelector(".headline").append(badge);
  const notice=document.createElement("p");notice.className="reason";notice.id="vision-chassis-link";
  document.getElementById("vision-message").after(notice);
  notice.textContent="视觉启停已联动底盘：实车模式启动后可能运动；停止视觉同时停止底盘。";
  let intent=null, stopping=false, linked=false;
  const robot=()=>frame.contentWindow?.RobotTrackingControl;
  const stamp=t=>Number(t?.image_stamp?.sec||0)*1e9+Number(t?.image_stamp?.nanosec||0);
  function cancel(message) {
    intent=null;robot()?.cancelStart();
    if(message)notice.textContent=message;
  }
  window.RobotVisionLink=Object.freeze({
    start(sendVision){
      if(stopping){notice.textContent="正在停止，请等待完成再启动";return;}
      cancel();const baseline=stamp(robot()?.snapshot()?.tracking), id=sendVision();
      if(!id){notice.textContent="视觉启动请求未发送，底盘保持停止";return;}
      intent={id,baseline,deadline:performance.now()+15000,accepted:false,running:false,sending:false};
      notice.textContent="正在启动视觉；等待本次启动确认和新的有效焊缝数据后启用底盘。";
    },
    async stop(sendVision){
      if(stopping)return;
      stopping=true;linked=false;cancel("正在停止底盘和视觉循迹…");
      let message="底盘已暂停；视觉停止请求已发送。";
      try {await robot()?.request("tracking.stop");if(!robot())throw Error("底盘页面未就绪");}
      catch(error){message="底盘停止未确认："+error.message+"；仍发送原视觉停止请求，请检查车辆。";}
      finally {
        if(!sendVision())message+=" 视觉停止请求未发出，请确认相机连接。";
        notice.textContent=message;stopping=false;
      }
    },
    receive(p){
      if(!intent)return;
      if(p.request_id===intent.id&&p.action==="vision.start"){
        if(p.status==="failed"){cancel("视觉启动失败，未启用底盘");return;}
        if(p.status==="completed")intent.accepted=true;
      }
      if(p.vision?.status==="running")intent.running=true;
      if(intent.accepted&&p.vision&&["failed","stopped","stopping"].includes(p.vision.status))cancel("视觉未运行，取消底盘启动");
    },
    reset(){
      cancel("连接已重置；不自动恢复底盘循迹，请重新点击启动。");
      if(linked){linked=false;robot()?.request("tracking.stop").catch(error=>{notice.textContent="连接重置，底盘停止待确认："+error.message;});}
    },
  });
  window.addEventListener("robot-explicit-stop",()=>{linked=false;cancel("底盘已人工停止；不会因待处理的视觉启动而再次运动。");});
  setInterval(async()=>{
    const current=intent;if(!current||current.sending)return;
    if(performance.now()>current.deadline){cancel("等待有效视觉超时，底盘未启动；请停止视觉并检查后重试。");return;}
    const t=robot()?.snapshot()?.tracking;
    if(!current.accepted||!current.running||!t?.fresh||!t.service_ready||!t.observation_valid||stamp(t)<=current.baseline)return;
    current.sending=true;
    try{
      await robot().request("tracking.start");
      if(intent!==current)return;
      notice.textContent=t.dry_run?"视觉与循迹预览已启动；预览模式不驱动车辆。":"视觉与底盘循迹已启用；请观察实测反馈。";intent=null;linked=true;
    }catch(error){if(intent===current){intent=null;notice.textContent="底盘未启动："+error.message+"。检查后重新停止/启动视觉，不自动重试。";}}
  },100);
  const post = data => frame.contentWindow?.postMessage(data, location.origin === "null" ? "*" : location.origin);
  frame.addEventListener("load", () => post({type:"robot-page-active", active:section.classList.contains("active")}));
  window.addEventListener("message", event => {
    if (event.source !== frame.contentWindow || event.data?.type !== "robot-link-state") return;
    badge.textContent = "底盘" + event.data.label;
    badge.className = "badge " + (event.data.connected ? "good" : "warn");
    if(!event.data.connected){linked=false;cancel("底盘连接中断；不自动恢复循迹，请检查后重新启动视觉。");}
  });
  document.getElementById("disconnect").addEventListener("click", () => post({type:"robot-link-disconnect"}));
  document.getElementById("connect").addEventListener("click", () => {
    try {
      const address = new URL(document.getElementById("endpoint").value);
      if (!window.RobotConnectionDefaults?.unified) address.port = new URL(endpoint).port;
      address.pathname = window.RobotConnectionDefaults?.unified ? "/ws/robot" : "/ws";
      address.search = "";address.hash = "";
      post({type:"robot-link-connect", endpoint:address.href});
    } catch (_) { /* 主页面会显示无效地址，不修改旧连接。 */ }
  });
  // 切到相机/采集页面时立即解除手动按钮；自动循迹不因看相机画面而被停止。
  document.querySelectorAll("nav [data-view]").forEach(button => button.addEventListener("click", () => {
    post({type:"robot-page-active", active:button.dataset.view === "chassis"});
  }));
})();
