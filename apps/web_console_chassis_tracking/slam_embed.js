"use strict";
(() => {
  const section=document.getElementById("view-slam"), frame=document.getElementById("slam-frame"), badge=document.getElementById("slam-link");
  document.getElementById("workspace").appendChild(section);
  const address=new URL(location.href);address.port="8082";address.pathname="/";address.search="";address.hash="";
  const origin=address.origin;
  const policy=document.createElement("meta");policy.httpEquiv="Content-Security-Policy";
  policy.content=`frame-src 'self' ${origin}`;document.head.append(policy);
  let heartbeat=0;
  const connection=(label,connected=false)=>{badge.textContent=label;badge.className=connected?"badge good":"badge warn";};
  const visible=()=>frame.contentWindow?.postMessage({type:"slam-page-active",active:section.classList.contains("active")&&!document.hidden},origin);
  const load=()=>{heartbeat=0;connection("正在加载 SLAM 页面…");frame.src=address.href;setTimeout(()=>{if(!heartbeat)connection("SLAM 页面未就绪：请检查整机启动日志 slam.log 及页面资源加载");},5000);};
  window.addEventListener("message",event=>{
    if(event.source!==frame.contentWindow || event.origin!==origin)return;
    if(event.data?.type==="slam-size"){
      const height=event.data.height;
      if(Number.isFinite(height) && height>=300 && height<=20000)frame.style.height=`${Math.ceil(height)}px`;
      return;
    }
    if(event.data?.type!=="slam-ready")return;
    heartbeat=Date.now();connection(event.data.connected?"SLAM 已连接":"SLAM 未连接",!!event.data.connected);visible();
  });
  frame.addEventListener("load",visible);
  document.querySelectorAll("nav [data-view]").forEach(button=>button.addEventListener("click",()=>setTimeout(visible,0)));
  document.addEventListener("visibilitychange",visible);
  setInterval(()=>{if(heartbeat && Date.now()-heartbeat>6000)connection("SLAM 未连接");},2000);
  load();
})();
