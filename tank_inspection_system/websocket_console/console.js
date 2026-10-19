/* Offline browser client for the existing tank_inspection_robot JSON protocol. */
"use strict";
(() => {
  const ROBOT = "tank_inspection_robot";
  const MAX_LOG = 200, MAX_MESSAGE = 1024 * 1024, MAX_BUFFER = 64 * 1024, STALE_MS = 3000;
  const $ = id => document.getElementById(id);
  const names = {fisheye:"鱼眼相机", mindvision:"Mindvision 工业相机", depth_camera:"D405 相机（RGB）", eddy_current:"涡流设备（聚合活性）", emat:"电磁超声", chassis:"底盘", lidar:"雷达点云", lidar_pose:"雷达位姿", eddy_raw:"涡流原始波形", eddy_mean:"涡流通道均值"};
  const state = {endpoint:"", connected:false, connectionCount:0, rx:0, tx:0, invalid:0, foreign:0, lastPacket:null, lastDevices:null, devices:{}, pose:null, bridge:null, controlEnabled:null, watchdogStops:null, response:null, logs:[], streams:{}};
  let socket = null, retryTimer = null, desired = false, retryDelay = 1000, sequence = 0;
  const object = value => value !== null && typeof value === "object" && !Array.isArray(value);
  const clip = (value, max = 256) => String(value == null ? "unavailable" : value).slice(0, max);
  const finite = value => typeof value === "number" && Number.isFinite(value);
  const now = () => performance.now();
  const age = timestamp => timestamp == null ? "unavailable" : ((now() - timestamp) / 1000).toFixed(1) + "s";
  const stamp = value => typeof value === "string" && /^-?\d{1,24}$/.test(value) ? value : "unavailable";
  const frameRate = times => {
    const window = times.filter(t => now() - t <= 2000);
    return window.length > 1 ? (window.length - 1) * 1000 / (window[window.length - 1] - window[0]) : 0;
  };
  function redact(value) {
    // Bound retained metadata too; a malformed peer must not fill the log with
    // arbitrary nested objects. Never keep JPEG bodies or credentials.
    let budget = 256;
    function copy(item, depth) {
      if (--budget < 0 || depth > 5) return "[TRUNCATED]";
      if (typeof item === "string") return clip(item, 512);
      if (Array.isArray(item)) return item.slice(0, 64).map(v => copy(v, depth + 1));
      if (!object(item)) return item;
      const out = Object.create(null);
      for (const key of Object.keys(item).slice(0, 32)) {
        out[key] = /token|password/i.test(key) ? "[REDACTED]" : key === "data" ? "[BODY OMITTED]" : copy(item[key], depth + 1);
        if (budget <= 0) break;
      }
      return out;
    }
    return copy(value, 0);
  }
  function log(kind, message, detail) {
    const entry = {at:new Date().toISOString(), endpoint:state.endpoint, kind, message:clip(message, 1024)};
    if (detail !== undefined) entry.detail = redact(detail);
    state.logs.push(entry);
    if (state.logs.length > MAX_LOG) state.logs.shift();
    const li = document.createElement("li");
    li.className = "log-" + kind;
    li.textContent = entry.at.slice(11, 23) + " " + entry.message + (entry.detail ? " " + clip(JSON.stringify(entry.detail), 1800) : "");
    $("log").appendChild(li);
    while ($("log").children.length > MAX_LOG) $("log").firstChild.remove();
    $("log").scrollTop = $("log").scrollHeight;
  }
  function notice(text) { $("notice").textContent = text; }
  function connection(text, mode) {
    $("connection").textContent = text;
    $("connection").className = "badge " + mode;
    $("connect").disabled = desired;
    $("disconnect").disabled = !desired && !socket;
    for (const id of ["check", "start-stream", "stop-stream", "stop"]) $(id).disabled = !state.connected;
    $("move").disabled = !state.connected || !$("enable-control").checked;
    $("endpoint").disabled = desired;
  }
  function invalidate(reason) {
    state.invalid++;
    log("error", reason);
  }
  function clearMeasurements() {
    state.devices = {}; state.pose = null; state.bridge = null; state.lastPacket = null; state.lastDevices = null;
    state.controlEnabled = null; state.watchdogStops = null; state.response = null;
    $("response").textContent = "尚无 response";
    for (const id of ["fisheye", "depth_camera"]) {
      const old = state.streams[id];
      if (old) old.generation++;
      state.streams[id] = {received:0, decoded:0, errors:0, last:null, source:null, times:[], decodedTimes:[], generation:0};
      const img = $(id + "-image"); img.onload = null; img.onerror = null; img.removeAttribute("src"); img.hidden = true;
      $(id + "-empty").hidden = false; $(id + "-empty").textContent = "尚未收到预览";
      $(id + "-meta").textContent = "等待 status_sensor";
    }
    render();
  }
  function send(kind, payload) {
    if (!socket || socket.readyState !== WebSocket.OPEN || !state.connected) {notice("尚未连接，未发送。");return false;}
    if (socket.bufferedAmount > MAX_BUFFER) {notice("发送缓冲积压，拒绝继续入队。");log("error", "send_buffer_full");return false;}
    const packet = {header:{robot_id:ROBOT, msg_type:kind}, payload};
    try {socket.send(JSON.stringify(packet));state.tx++;log("tx", "发送 " + kind, packet);return true;}
    catch (error) {notice("发送失败：" + clip(error.message));log("error", "send_failed");return false;}
  }
  function withToken(payload) {
    if ($("control-token").value) payload.token = $("control-token").value;
    return payload;
  }
  function openSocket() {
    if (!desired) return;
    clearMeasurements();
    const own = ++sequence;
    let ws;
    try {ws = new WebSocket(state.endpoint);socket = ws;}
    catch (error) {desired = false;connection("地址无效", "bad");notice(clip(error.message));return;}
    connection("连接中…", "wait");
    ws.onopen = () => {
      if (own !== sequence || !desired) {ws.close();return;}
      state.connected = true;state.connectionCount++;retryDelay = 1000;
      connection("已连接", "good");notice("");log("info", "连接 " + state.endpoint);
      send("check", {});
    };
    ws.onmessage = event => {if (own === sequence) receive(event.data);};
    ws.onerror = () => {if (own === sequence) notice("连接失败，请核对地址、端口和板端桥接进程。");};
    ws.onclose = event => {
      if (own !== sequence) return;
      state.connected = false;socket = null;
      log("info", "连接关闭", {code:event.code, reason:event.reason});
      if (desired && $("reconnect").checked) {
        connection("等待重连…", "wait");
        retryTimer = setTimeout(() => {retryTimer = null;openSocket();}, retryDelay);
        retryDelay = Math.min(retryDelay * 2, 5000);
      } else {desired = false;connection("已断开", "wait");}
      render();
    };
  }
  function connect() {
    if (desired) return;
    try {
      const url = new URL($("endpoint").value.trim());
      if (!["ws:", "wss:"].includes(url.protocol) || url.username || url.password || url.hash) throw new Error("请输入 ws:// 或 wss:// 地址，不携带口令或片段。");
      state.endpoint = url.href;
    } catch (error) {notice(clip(error.message));return;}
    desired = true;retryDelay = 1000;
    openSocket();
  }
  function disconnect() {
    // Only a user-enabled command session sends a best-effort stop on disconnect.
    if (state.connected && $("enable-control").checked) send("command", withToken({agv:{command:"stop"}}));
    desired = false;clearTimeout(retryTimer);retryTimer = null;sequence++;
    if (socket) {socket.onclose = null;socket.close(1000, "operator_disconnect");socket = null;}
    state.connected = false;connection("已断开", "wait");render();
    log("info", "用户断开；自动重连已取消");
  }
  function updateDevices(devices) {
    for (const [key, value] of Object.entries(devices)) if (Object.hasOwn(names, key) && object(value)) state.devices[key] = redact(value);
    state.lastDevices = now();
  }
  function receive(raw) {
    if (typeof raw !== "string" || raw.length > MAX_MESSAGE) {invalidate("non_text_or_oversize_message");return;}
    let packet;
    try {packet = JSON.parse(raw);} catch (_) {invalidate("invalid_json");return;}
    if (!object(packet) || !object(packet.header) || !object(packet.payload) || typeof packet.header.msg_type !== "string") {invalidate("invalid_envelope");return;}
    if (packet.header.robot_id !== ROBOT) {state.foreign++;return;}
    state.rx++;state.lastPacket = now();
    const payload = packet.payload, kind = packet.header.msg_type;
    if (kind === "status_update") {
      if (object(payload.devices)) {
        updateDevices(payload.devices);
        state.pose = object(payload.lidar) ? redact(payload.lidar) : null;
      }
      if (object(payload.bridge)) state.bridge = redact(payload.bridge);
      if (typeof payload.control_enabled === "boolean") state.controlEnabled = payload.control_enabled;
      if (finite(payload.watchdog_stops)) state.watchdogStops = payload.watchdog_stops;
    } else if (kind === "response") {
      state.response = redact(payload);$("response").textContent = JSON.stringify(state.response, null, 2);
      const devs = {};
      for (const key of Object.keys(names)) if (object(payload[key]) && payload[key].command === "check") devs[key] = payload[key];
      if (Object.keys(devs).length) updateDevices(devs);
      log("rx", "收到 response（共享回包）", state.response);
    } else if (kind === "status_sensor" && object(payload.video)) {
      receiveVideo(payload.video);
    } else {log("info", "收到其他类型 " + clip(kind));}
  }
  function receiveVideo(video) {
    const id = video.camera_id;
    if (!Object.hasOwn(state.streams, id)) {invalidate("unknown_video_camera_id");return;}
    if (video.encoding !== "jpeg_base64" || typeof video.data !== "string" || video.data.length > 800000 || !/^[A-Za-z0-9+/]+={0,2}$/.test(video.data) || !Number.isInteger(video.width) || !Number.isInteger(video.height) || video.width <= 0 || video.height <= 0 || video.width > 4096 || video.height > 4096) {invalidate("invalid_video_payload");return;}
    const stream = state.streams[id], generation = ++stream.generation;
    stream.received++;stream.last = now();stream.times.push(stream.last);
    if (stream.times.length > 60) stream.times.shift();
    stream.source = redact(video);
    delete stream.source.data;
    const img = $(id + "-image");
    img.onload = () => {
      if (state.streams[id] !== stream || generation !== stream.generation) return;
      if (img.naturalWidth !== video.width || img.naturalHeight !== video.height) {
        stream.errors++;img.hidden = true;$(id + "-empty").hidden = false;$(id + "-empty").textContent = "JPEG 尺寸与回包不符";log("error", "jpeg_dimensions_mismatch " + id);return;
      }
      stream.decoded++;stream.decodedTimes.push(now());
      if (stream.decodedTimes.length > 60) stream.decodedTimes.shift();
      img.hidden = false;$(id + "-empty").hidden = true;
    };
    img.onerror = () => {
      if (state.streams[id] !== stream || generation !== stream.generation) return;
      stream.errors++;img.hidden = true;$(id + "-empty").hidden = false;$(id + "-empty").textContent = "JPEG 解码失败";log("error", "jpeg_decode_failed " + id);
    };
    img.src = "data:image/jpeg;base64," + video.data;
  }
  const deviceNodes = {};
  for (const [key, name] of Object.entries(names)) {
    const card = document.createElement("div");card.id = "device-" + key;card.className = "device wait";
    const title = document.createElement("span");title.className = "device-name";title.textContent = name;
    const status = document.createElement("span");status.className = "device-status";
    const reason = document.createElement("div");reason.className = "device-reason";
    const detail = document.createElement("div");detail.className = "device-detail";
    card.append(title, status, reason, detail);$("devices").appendChild(card);deviceNodes[key] = {card, status, reason, detail};
  }
  function render() {
    for (const key of ["rx", "tx", "invalid", "foreign"]) $(key + "-count").textContent = state[key];
    $("last-received").textContent = state.lastPacket === null ? "尚无回包" : "距收到回包 " + age(state.lastPacket);
    const fresh = state.connected && state.lastDevices !== null && now() - state.lastDevices <= STALE_MS;
    for (const [key, node] of Object.entries(deviceNodes)) {
      const d = state.devices[key], valid = fresh && d;
      const status = valid ? d.status : "unverified";
      node.card.className = "device " + (status === "verified" ? "good" : status === "failed" ? "bad" : "wait");
      node.status.textContent = !valid && d ? "显示过期" : status === "verified" ? "数据有效" : status === "failed" ? "数据异常" : status === "unverified" ? "未验证" : "未知状态";
      node.reason.textContent = d ? clip(d.reason) : "等待板端自检";
      node.detail.textContent = d ? "topic: " + clip(d.topic) + "\ntype: " + clip(d.message_type) + "\n收到/有效/无效: " + clip(d.received) + "/" + clip(d.accepted) + "/" + clip(d.invalid) + "\nstamp_ns: " + stamp(d.stamp_ns) + "\n时间来源: " + clip(d.timestamp_source) : "";
    }
    for (const [id, stream] of Object.entries(state.streams)) {
      $(id + "-rate").textContent = "显示 " + frameRate(stream.decodedTimes).toFixed(1) + " / 收到 " + frameRate(stream.times).toFixed(1) + " FPS";
      if (stream.source) {
        const v = stream.source;
        const current = state.connected && now() - stream.last <= STALE_MS && state.devices[id]?.status === "verified";
        $(id + "-meta").textContent = (current ? "收到预览" : "预览已过期或设备未验证") + " · " + age(stream.last) + "前\n" + v.width + "×" + v.height + " · source " + clip(v.source_encoding) + " · " + clip(v.frame_id) + "\nstamp_ns: " + stamp(v.stamp_ns) + "\n收到/解码/错误: " + stream.received + "/" + stream.decoded + "/" + stream.errors + " · 板端 latest 覆盖: " + clip(v.preview_dropped);
      }
    }
    const pose = fresh ? state.pose : null, metric = pose && pose.position_unit === "m" && pose.angle_unit === "rad" && [pose.x, pose.y, pose.z, pose.angle].every(finite);
    for (const key of ["x", "y", "z", "angle"]) $("pose-" + key).textContent = metric ? pose[key].toFixed(4) : "—";
    $("pose-status").textContent = metric ? "收到 Odometry · 资产坐标未验证" : "未收到当前有效位姿";
    $("pose-meta").textContent = metric ? "frame: " + clip(pose.frame_id) + " → " + clip(pose.child_frame_id) + " · stamp_ns: " + stamp(pose.stamp_ns) + "\n四元数: " + JSON.stringify(pose.orientation || null) : "坐标系与源时间戳待有效回包。资产坐标与标定状态未验证。";
    $("bridge-meta").textContent = state.bridge ? "桥接 command_dropped=" + clip(state.bridge.command_dropped) + " · transport_dropped=" + clip(state.bridge.transport_dropped) + "（含无连接期间；不等同本页面丢帧）" : "等待桥接统计";
    $("control-status").textContent = "板端最后报告控制 " + (state.controlEnabled === null ? "未知" : state.controlEnabled ? "已启用" : "已关闭") + " · watchdog_stops=" + clip(state.watchdogStops) + " · 本页面命令 " + ($("enable-control").checked ? "允许人工发送" : "关闭");
  }
  function snapshot() {
    return {schema_version:"tank_websocket_console_v1", exported_at:new Date().toISOString(), endpoint:state.endpoint, robot_id:ROBOT, connected:state.connected, connection_count:state.connectionCount,
      counts_scope:"page_lifetime_across_connections", counts:{received:state.rx, sent:state.tx, invalid:state.invalid, foreign:state.foreign}, device_display_fresh:state.connected && state.lastDevices !== null && now() - state.lastDevices <= STALE_MS, devices:state.devices,
      pose:state.pose, bridge:state.bridge, last_response:state.response, streams:Object.fromEntries(Object.entries(state.streams).map(([id, s]) => [id,{received:s.received, decoded:s.decoded, received_fps:frameRate(s.times), displayed_fps:frameRate(s.decodedTimes), fps_window_sec:2, decode_errors:s.errors, unrendered_or_pending:s.received - s.decoded - s.errors, last_source:s.source, receipt_age_seconds:s.last === null ? null : (now() - s.last) / 1000}])),
      hardware_revalidated:false, production_ready:false, response_correlation:"unavailable_no_request_id", logs:state.logs};
  }
  $("connect").onclick = connect;$("disconnect").onclick = disconnect;
  $("check").onclick = () => send("check", {});
  $("start-stream").onclick = () => send("command", {camera:{command:"start_stream"}});
  $("stop-stream").onclick = () => send("command", {camera:{command:"stop_stream"}});
  $("enable-control").onchange = () => {$("move").disabled = !state.connected || !$("enable-control").checked;render();};
  $("reconnect").onchange = () => {if (!$("reconnect").checked && !state.connected && retryTimer) disconnect();};
  $("move").onclick = () => {
    if (!$("enable-control").checked) return;
    const parameters = {}, limits = {vx:.2, vy:.2, wz:.5};
    for (const key of Object.keys(limits)) {
      const text = $(key).value.trim(), value = Number(text);
      if (!text || !Number.isFinite(value) || Math.abs(value) > limits[key]) {notice("速度无效或超出本页面测试上限：" + key);return;}
      parameters[key] = value;
    }
    send("command", withToken({agv:{command:"move", parameters}}));
  };
  $("stop").onclick = () => send("command", withToken({agv:{command:"stop"}}));
  $("clear-log").onclick = () => {state.logs = [];$("log").replaceChildren();};
  $("export").onclick = () => {
    const url = URL.createObjectURL(new Blob([JSON.stringify(snapshot(), null, 2)], {type:"application/json"}));
    const a = document.createElement("a");a.href = url;a.download = "tank-websocket-" + new Date().toISOString().replace(/[:.]/g, "-") + ".json";a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  const query = new URLSearchParams(location.search);
  if (query.has("ws")) $("endpoint").value = query.get("ws");
  else if (location.hostname) $("endpoint").value = "ws://" + (location.hostname.includes(":") ? "[" + location.hostname + "]" : location.hostname) + ":9001";
  clearMeasurements();connection("未连接", "wait");
  const renderTimer = setInterval(render, 250);
  window.addEventListener("pagehide", () => {disconnect();clearInterval(renderTimer);});
  // Read-only diagnostic/export hook; outgoing traffic always comes from UI actions.
  window.tankConsole = Object.freeze({snapshot});
})();
