/* 保留原网页 8080 / 通信 9001；新功能共用原 WebSocket 端口。 */
"use strict";
(() => {
  const host=location.hostname.includes(":")?`[${location.hostname}]`:(location.hostname||"127.0.0.1");
  const scheme=location.protocol==="https:"?"wss:":"ws:";
  window.RobotConnectionDefaults=Object.freeze({unified:true,
    inspection:`${scheme}//${host}:9001/ws/inspection`,robot:`${scheme}//${host}:9001/ws/robot`,
    emat:`${scheme}//${host}:9001/ws/emat`});
})();
