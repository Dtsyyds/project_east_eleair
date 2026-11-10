/* 遥控接管通知：只读反馈→一次提示，不获取权限，也不发送任何恢复命令。 */
"use strict";
(() => {
  window.createTakeoverNotice = (bannerParent) => {
    const banner=document.createElement("p");
    banner.id="remote-takeover-banner";banner.className="remote-takeover-banner";
    banner.hidden=true;banner.setAttribute("role","status");bannerParent.append(banner);
    const dialog=document.createElement("dialog");
    dialog.id="remote-takeover-dialog";dialog.className="remote-takeover-dialog";
    dialog.setAttribute("role","alertdialog");dialog.setAttribute("aria-modal","false");
    dialog.setAttribute("aria-labelledby","remote-takeover-title");
    const title=document.createElement("h2");title.id="remote-takeover-title";title.textContent="遥控器接管提示";
    const domain=document.createElement("p");domain.id="remote-takeover-domain";
    const detail=document.createElement("p");detail.id="remote-takeover-detail";
    const hint=document.createElement("p");hint.textContent="此提示只确认已知晓，不发送停止、交还或恢复自动控制指令。请以实际控制权和安全状态为准。";
    const ack=document.createElement("button");ack.id="remote-takeover-ack";ack.type="button";
    ack.textContent="知道了（不恢复自动控制）";
    ack.addEventListener("click",event=>{event.stopPropagation();dialog.close();});
    // 不让弹窗内键盘操作冒泡成 WASD 等运动按键，keyup 仍可撤销先前按住的意图。
    dialog.addEventListener("keydown",event=>event.stopPropagation());
    dialog.append(title,domain,detail,hint,ack);document.body.append(dialog);
    const labels={chassis:"底盘",probe:"探头电机"};
    const last={chassis:null,probe:null};let seen=false, episodes=0;
    function update(snapshot) {
      const current={}, entered=[];
      for(const name of Object.keys(labels)) {
        const state=snapshot?.[name];current[name]=["remote","released"].includes(state)?state:"unknown";
        if(current[name]==="remote"){
          if(last[name]!==true)entered.push(labels[name]);last[name]=true;
        }else if(current[name]==="released")last[name]=false;
        // 反馈过期或断线只表示未知，不当成退出接管，避免恢复反馈后重复弹窗。
      }
      const active=Object.keys(labels).filter(n=>current[n]==="remote").map(n=>labels[n]);
      const uncertain=Object.keys(labels).filter(n=>current[n]==="unknown"&&last[n]===true).map(n=>labels[n]);
      if(entered.length){
        seen=true;domain.textContent="遥控器已接管："+entered.join("、");
        dialog.dataset.episode=String(++episodes);
        // 非阻塞弹窗：保持后台刷新和停止控件可用，不使用阻塞脚本的 alert()。
        if(!dialog.open)dialog.show();
      }
      let text="";
      if(active.length)text="遥控器正在控制："+active.join("、")+"。关闭提示不会恢复自动控制。";
      else if(uncertain.length)text="遥控接管状态待确认："+uncertain.join("、")+"反馈未知，不能据此认定已经交还。";
      else if(seen)text="最近一次遥控接管已结束；实际能否恢复自动控制仍由底盘安全状态和启用流程决定。";
      banner.hidden=!text;banner.textContent=text;detail.textContent=text;
    }
    return Object.freeze({update});
  };
})();
