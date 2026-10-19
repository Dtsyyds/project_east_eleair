/* Offline-first browser client for the tank inspection robot JSON protocol. */
"use strict";
(() => {
  const C = window.TankConsoleCore;
  if (!C) throw new Error("console_core.js was not loaded");
  const DEFAULT_ROBOT = "tank_inspection_robot";
  const MAX_LOG = 200, MAX_MESSAGE = 1024 * 1024, MAX_BUFFER = 64 * 1024;
  const STREAM_IDS = ["mindvision", "fisheye", "depth_camera"];
  const TASK_STAGES = ["preflighting", "ready", "starting", "recording", "stopping", "finalizing", "complete"];
  const SIDE_EFFECTS = new Set(["collection.start", "collection.stop", "emat.measure", "replay.start", "replay.pause", "replay.resume", "replay.stop", "evidence.save_displayed", "evidence.save_next", "event.mark", "chassis.move", "chassis.stop"]);
  const $ = id => document.getElementById(id);
  const monoNow = () => performance.now();
  const isoNow = () => new Date().toISOString();
  const ageText = value => value == null ? "未知" : Math.max(0, (monoNow() - value) / 1000).toFixed(1) + " 秒";
  const valueText = value => value == null || value === "" ? "未提供" : String(value);
  const bytesText = value => {
    if (!C.finite(value) || value < 0) return "未提供";
    const units = ["B", "KiB", "MiB", "GiB", "TiB"];
    let number = value, index = 0;
    while (number >= 1024 && index < units.length - 1) {number /= 1024;index++;}
    return number.toFixed(index ? 1 : 0) + " " + units[index];
  };
  const newClientId = () => crypto.randomUUID ? crypto.randomUUID() : "client-" + Date.now() + "-" + Math.random().toString(16).slice(2);
  const clientId = newClientId();

  function emptyStream(id, previous) {
    return {
      id, connectionId:state ? state.connectionId : 0, received:0, displayed:0, decodeErrors:0,
      dimensionErrors:0, clientReplaced:0, hiddenDropped:0, outOfOrder:0, crossConnectionDiscarded:0,
      serverDropped:null, receiveTimes:[], displayTimes:[], decoding:false, pending:null, hiddenPending:null,
      lastSourceStamp:null, lastReceived:null, lastDisplayed:previous?.lastDisplayed || null,
      overlayVisible:previous?.overlayVisible !== false, latestResultState:"none"
    };
  }

  const state = {
    endpoint:"", robotId:DEFAULT_ROBOT, robotSource:"default", mode:null, connected:false, desired:false,
    connectionId:0, connectionCount:0, rx:0, tx:0, invalid:0, foreign:0, lastPacket:null,
    capabilities:null, capabilitiesAt:null, devices:Object.create(null), streamHealth:Object.create(null),
    pose:null, trajectory:[], trajectoryFrame:null, bridge:null, control:null, storage:null, task:null, measurement:null,
    tracking:null, trackingByFrame:new Map(), events:[], history:[], alerts:new Map(), acknowledged:new Set(),
    response:null, logs:[], requests:new Map(), requestHistory:[], uncertainActions:new Set(),
    eddySamples:[], eddyMeta:null, streams:Object.create(null), lifecycleDiscards:0
  };
  let socket = null, retryTimer = null, retryDelay = 1000, renderTimer = null, sequence = 0;
  let pageReconnectWanted = false, restoringReadStateFor = 0;
  STREAM_IDS.forEach(id => {state.streams[id] = emptyStream(id, null);});

  function setBadge(id, text, mode = "muted") {
    const node = $(id); node.textContent = text; node.className = "badge " + mode;
  }

  function notice(text) {$("notice").textContent = text || "";}

  function log(kind, message, detail) {
    const entry = {at:isoNow(), endpoint:state.endpoint, connection_id:state.connectionId, kind, message:C.clip(message, 1024, "")};
    if (detail !== undefined) entry.detail = C.redact(detail);
    C.boundedPush(state.logs, entry, MAX_LOG);
    const li = document.createElement("li");
    li.className = "log-" + kind;
    li.textContent = entry.at.slice(11, 23) + " " + entry.message + (entry.detail ? " " + C.clip(JSON.stringify(entry.detail), 1800, "") : "");
    $("log").appendChild(li);
    while ($("log").children.length > MAX_LOG) $("log").firstChild.remove();
    $("log").scrollTop = $("log").scrollHeight;
  }

  function invalidate(reason) {state.invalid++;log("error", reason);}

  function record(data, valid = true, reason = "") {
    const result = C.makeRecord(data, monoNow(), valid, reason);
    result.connectionId = state.connectionId;
    return result;
  }

  function currentRecord(item) {return item && item.connectionId === state.connectionId ? item : null;}

  function resetConnectionScope() {
    state.connectionId++;
    state.capabilities = null; state.capabilitiesAt = null; state.mode = null; state.bridge = null;
    state.task = null; state.measurement = null; state.storage = null;
    state.response = null; state.requests.clear(); state.uncertainActions.clear(); restoringReadStateFor = 0;
    $("response").textContent = "尚无 response";
    $("capabilities").textContent = "板端尚未提供 capabilities";
    for (const id of STREAM_IDS) state.streams[id] = emptyStream(id, state.streams[id]);
    state.trackingByFrame.clear(); state.trajectory = []; state.trajectoryFrame = null;
    $("control-token").value = ""; $("enable-control").checked = false;
  }

  function updateConnectionUi(text, mode) {
    setBadge("connection", text, mode);
    $("connect").disabled = state.desired;
    $("disconnect").disabled = !state.desired && !socket;
    $("endpoint").disabled = state.desired;
    for (const id of ["check", "start-stream", "stop-stream"]) $(id).disabled = !state.connected;
  }

  function sendEnvelope(kind, payload, description) {
    if (!socket || socket.readyState !== WebSocket.OPEN || !state.connected) {notice("尚未连接，未发送。");return false;}
    if (socket.bufferedAmount > MAX_BUFFER) {notice("发送缓冲积压，拒绝继续入队。");log("error", "send_buffer_full");return false;}
    const packet = {header:{robot_id:state.robotId, msg_type:kind}, payload};
    try {
      socket.send(JSON.stringify(packet)); state.tx++;
      log("tx", description || "发送 " + kind, packet); return true;
    } catch (error) {notice("发送失败：" + C.clip(error.message));log("error", "send_failed");return false;}
  }

  function sendLegacy(kind, payload) {return sendEnvelope(kind, payload, "发送兼容协议 " + kind);}

  function operationGate(action) {return C.operationGate(state.capabilities, action, state.connected);}

  function requestId() {return state.connectionId + "-" + Date.now().toString(36) + "-" + Math.random().toString(36).slice(2, 9);}

  function actionInFlight(action) {
    for (const value of state.requests.values()) if (value.action === action && !["completed", "failed"].includes(value.status)) return true;
    return state.uncertainActions.has(action);
  }

  function sendOperation(action, parameters = {}, options = {}) {
    const gate = operationGate(action);
    if (!gate.allowed) {notice(gate.reason);return null;}
    if (SIDE_EFFECTS.has(action) && actionInFlight(action)) {notice("该操作已有请求在处理或结果未知，未重复发送。");return null;}
    const id = requestId();
    const payload = {action, request_id:id, client_id:clientId, parameters:C.redact(parameters)};
    if (options.token) payload.token = options.token;
    if (!sendEnvelope(gate.spec.msg_type, payload, "发送操作 " + action + " request_id=" + id)) return null;
    if(action==="emat.measure") state.measurement=record({status:"running",message:"正在测厚…",progress_percent:0,request_id:id,source:parameters.source});
    const timeoutMs = C.finite(gate.spec.timeout_ms) ? Math.min(120000, Math.max(2000, gate.spec.timeout_ms)) : 10000;
    const request = {request_id:id, action, status:"sent", sentAt:monoNow(), timeoutAt:monoNow() + timeoutMs, sideEffect:SIDE_EFFECTS.has(action), job_id:null};
    state.requests.set(id, request); C.boundedPush(state.requestHistory, C.redact(request), 100);
    window.setTimeout(() => timeoutRequest(id), timeoutMs + 20);
    renderButtons(); return id;
  }

  function timeoutRequest(id) {
    const request = state.requests.get(id);
    if (!request || ["completed", "failed"].includes(request.status) || monoNow() < request.timeoutAt) return;
    request.status = "unknown"; request.finishedAt = monoNow();
    if (request.sideEffect) state.uncertainActions.add(request.action);
    log("error", "请求超时，结果未知；不自动重试 " + request.action, {request_id:id});
    state.requests.delete(id);
    if (request.action.startsWith("collection.") && operationGate("collection.status").allowed && !actionInFlight("collection.status")) sendOperation("collection.status", {});
    renderButtons();
  }

  function settleRequest(payload) {
    if (!C.isObject(payload) || typeof payload.request_id !== "string") return false;
    const request = state.requests.get(payload.request_id);
    if (!request) {log("info", "收到未知或跨客户端 request_id", {request_id:payload.request_id, status:payload.status});return false;}
    const status = String(payload.status || payload.state || "unknown").toLowerCase();
    request.status = ["accepted", "running", "completed", "failed", "unknown"].includes(status) ? status : "unknown";
    if (typeof payload.job_id === "string") request.job_id = payload.job_id;
    if (["completed", "failed"].includes(request.status)) {
      request.finishedAt = monoNow(); state.requests.delete(payload.request_id); state.uncertainActions.delete(request.action);
    }
    C.boundedPush(state.requestHistory, C.redact(Object.assign({}, request, {response:payload})), 100);
    return true;
  }

  function openSocket() {
    if (!state.desired) return;
    resetConnectionScope();
    const own = ++sequence; let ws;
    try {ws = new WebSocket(state.endpoint);socket = ws;}
    catch (error) {state.desired = false;updateConnectionUi("地址无效", "bad");notice(C.clip(error.message));return;}
    updateConnectionUi("连接中…", "warn"); render();
    ws.onopen = () => {
      if (own !== sequence || !state.desired) {ws.close();return;}
      state.connected = true; state.connectionCount++; retryDelay = 1000;
      updateConnectionUi("已连接", "good"); notice(""); log("info", "连接 " + state.endpoint);
      sendLegacy("check", {});
    };
    ws.onmessage = event => {if (own === sequence) receive(event.data);};
    ws.onerror = () => {if (own === sequence) notice("连接失败，请核对地址、端口和板端桥接进程。");};
    ws.onclose = event => {
      if (own !== sequence) return;
      state.connected = false; socket = null; $("control-token").value = ""; $("enable-control").checked = false;
      log("info", "连接关闭", {code:event.code, reason:event.reason});
      if (state.desired && $("reconnect").checked) {
        updateConnectionUi("等待重连…", "warn");
        retryTimer = window.setTimeout(() => {retryTimer = null;openSocket();}, retryDelay);
        retryDelay = Math.min(retryDelay * 2, 5000);
      } else {state.desired = false;updateConnectionUi("已断开", "warn");}
      render();
    };
  }

  function connect() {
    if (state.desired) return;
    try {state.endpoint = C.safeEndpoint($("endpoint").value, location.protocol);}
    catch (error) {notice(C.clip(error.message));return;}
    if ($("remember-endpoint").checked) {try {localStorage.setItem("tank_console_endpoint", state.endpoint);} catch (_) {}}
    state.desired = true; retryDelay = 1000; openSocket();
  }

  function closeSocket(userInitiated, preserveWanted = false) {
    if (state.connected && $("enable-control").checked && controlGate().allowed) sendControl(true);
    if (!preserveWanted) state.desired = false;
    clearTimeout(retryTimer); retryTimer = null; sequence++;
    if (socket) {socket.onclose = null;socket.close(1000, userInitiated ? "operator_disconnect" : "page_hidden");socket = null;}
    state.connected = false; $("control-token").value = ""; $("enable-control").checked = false;
    updateConnectionUi("已断开", "warn");render();
    if (userInitiated) log("info", "用户断开；自动重连已取消");
  }

  function disconnect() {closeSocket(true, false);}

  function acceptCapabilities(value) {
    if (!C.isObject(value)) return;
    state.capabilities = C.redact(value); state.capabilitiesAt = monoNow();
    if (typeof value.robot_id === "string" && /^[A-Za-z0-9_.:-]{1,80}$/.test(value.robot_id)) {state.robotId = value.robot_id;state.robotSource = "capabilities";}
    if (typeof value.mode === "string") state.mode = value.mode;
    $("capabilities").textContent = JSON.stringify(state.capabilities, null, 2);
    const version = value.protocol_version ?? value.version;
    setBadge("protocol-version", version == null ? "协议版本未提供" : "协议 v" + C.clip(version), version == null ? "warn" : "info");
    updateProfiles(); renderButtons();
    if (restoringReadStateFor !== state.connectionId) {
      restoringReadStateFor = state.connectionId;
      if (operationGate("collection.status").allowed) sendOperation("collection.status", {});
      if (operationGate("history.list").allowed) sendOperation("history.list", {});
      if (operationGate("replay.status").allowed) sendOperation("replay.status", {});
    }
  }

  function updateProfiles() {
    const select = $("profile"); const current = select.value;
    select.replaceChildren();
    const profiles = Array.isArray(state.capabilities?.collection?.profiles) ? state.capabilities.collection.profiles : [];
    if (!profiles.length) {const option = document.createElement("option");option.value = "";option.textContent = "板端未提供";select.appendChild(option);return;}
    for (const item of profiles.slice(0, 50)) {
      const value = typeof item === "string" ? item : item?.id;
      if (typeof value !== "string") continue;
      const option = document.createElement("option");option.value = value;option.textContent = typeof item === "string" ? item : C.clip(item.label || item.id);select.appendChild(option);
    }
    if ([...select.options].some(option => option.value === current)) select.value = current;
  }

  function updateDevices(devices) {
    for (const [key, value] of Object.entries(devices)) if (Object.hasOwn(C.PHYSICAL_DEVICES, key) && C.isObject(value)) state.devices[key] = record(value, value.valid !== false, value.reason || "");
  }

  function updateStreamHealth(streams) {
    for (const [key, value] of Object.entries(streams)) if (C.isObject(value)) state.streamHealth[key] = record(value, value.valid !== false, value.reason || "");
  }

  function acceptPose(value) {
    if (!C.isObject(value)) return;
    const metric = value.position_unit === "m" && value.angle_unit === "rad" && [value.x, value.y, value.z, value.angle].every(C.finite);
    const hasFrames = typeof value.frame_id === "string" && value.frame_id.length > 0;
    const hasStamp = Boolean(C.stamp(value.stamp_ns));
    state.pose = record(value, metric && hasFrames && hasStamp && value.valid !== false, metric ? (!hasFrames ? "缺少坐标系" : !hasStamp ? "源时间无效" : "") : "坐标或单位无效");
    if (state.pose.valid) {
      if (state.trajectoryFrame !== value.frame_id) {state.trajectory = [];state.trajectoryFrame = value.frame_id;}
      C.boundedPush(state.trajectory, {x:value.x,y:value.y,stamp_ns:value.stamp_ns}, 600);
    }
  }

  function acceptTask(value) {
    if (!C.isObject(value)) return;
    const previous=state.task?.data;
    if(value.service_id===previous?.service_id&&C.finite(value.revision)&&C.finite(previous?.revision)&&value.revision<previous.revision)return;
    state.task = record(value, value.valid !== false, value.error || "");
    if (typeof value.mode === "string") state.mode = value.mode;
    for (const action of ["collection.start", "collection.stop"]) state.uncertainActions.delete(action);
  }

  function acceptStorage(value) {if (C.isObject(value)) state.storage = record(value, value.valid !== false, value.error || "");}

  function acceptAlerts(values) {
    if (!Array.isArray(values)) return;
    for (const item of values.slice(0, 100)) {
      if (!C.isObject(item)) continue;
      const id = C.clip(item.alert_id || item.id || item.code || JSON.stringify([item.category,item.message]), 160, "unknown");
      const prior = state.alerts.get(id);
      const merged = Object.assign({}, prior?.data || {}, C.redact(item));
      if (prior && item.count == null) merged.count = (prior.data.count || 1) + 1;
      state.alerts.set(id, record(merged, true));
    }
    while (state.alerts.size > 100) state.alerts.delete(state.alerts.keys().next().value);
  }

  function acceptEvents(values) {
    const list = Array.isArray(values) ? values : [values];
    for (const item of list) {
      if (!C.isObject(item)) continue;
      const id = item.event_id || item.request_id;
      if (typeof id !== "string") continue;
      const index = state.events.findIndex(existing => existing.event_id === id);
      const value = C.redact(item);
      if (index >= 0) state.events[index] = Object.assign({}, state.events[index], value);
      else state.events.unshift(value);
      for (const action of ["evidence.save_displayed", "evidence.save_next", "event.mark"]) state.uncertainActions.delete(action);
    }
    state.events = state.events.slice(0, 100);
  }

  function acceptHistory(values) {if (Array.isArray(values)) state.history = C.redact(values.slice(0, 100));}

  function acceptEddy(value) {
    if (!C.isObject(value)) return;
    const channels = Array.isArray(value.channel_mean) ? value.channel_mean.filter(C.finite).slice(0, 32) : null;
    if (!channels?.length) return;
    C.boundedPush(state.eddySamples, {at:monoNow(), values:channels}, 600); state.eddyMeta = record(value, true);
  }

  function acceptTracking(value) {
    if (!C.isObject(value)) return;
    const validStates = Object.hasOwn(C.TRACKING_LABELS, String(value.status || "").toUpperCase());
    state.tracking = record(value, validStates && value.valid !== false, validStates ? "" : "未知巡迹状态");
    const key = C.frameKey(value, state.robotId);
    if (key) {
      state.trackingByFrame.set(key, state.tracking);
      while (state.trackingByFrame.size > 80) state.trackingByFrame.delete(state.trackingByFrame.keys().next().value);
      for (const id of STREAM_IDS) {
        const displayed = state.streams[id].lastDisplayed;
        if (displayed && C.frameKey(displayed, state.robotId) === key) drawOverlay(id, displayed, state.tracking);
      }
    }
  }

  function receive(raw) {
    if (typeof raw !== "string" || raw.length > MAX_MESSAGE) {invalidate("non_text_or_oversize_message");return;}
    let packet;
    try {packet = JSON.parse(raw);} catch (_) {invalidate("invalid_json");return;}
    if (!C.isObject(packet) || !C.isObject(packet.header) || !C.isObject(packet.payload) || typeof packet.header.msg_type !== "string") {invalidate("invalid_envelope");return;}
    if (packet.header.robot_id !== state.robotId && packet.payload.robot_id !== state.robotId) {state.foreign++;return;}
    state.rx++; state.lastPacket = monoNow();
    const payload = packet.payload, kind = packet.header.msg_type;
    window.TankReplay.receive(packet);
    if(kind==="replay_sensor")return;
    if (C.isObject(payload.capabilities)) acceptCapabilities(payload.capabilities);
    if (C.isObject(payload.task)) acceptTask(payload.task);
    if (C.isObject(payload.storage)) acceptStorage(payload.storage);
    if (C.isObject(payload.measurement)) {
      const value=payload.measurement,previous=state.measurement?.data;
      if(!(value.service_id===previous?.service_id&&C.finite(value.revision)&&C.finite(previous?.revision)&&value.revision<previous.revision)) {
        state.measurement = record(value, true);
        if (["completed", "failed", "idle"].includes(value.status)) state.uncertainActions.delete("emat.measure");
      }
    }
    if (Array.isArray(payload.alerts)) acceptAlerts(payload.alerts);
    if (payload.event || payload.events) acceptEvents(payload.event || payload.events);
    if (Array.isArray(payload.history)) acceptHistory(payload.history);
    if (C.isObject(payload.tracking_result)) acceptTracking(payload.tracking_result);
    if (C.isObject(payload.eddy_mean)) acceptEddy(payload.eddy_mean);
    if (C.isObject(payload.eca?.mean)) acceptEddy(payload.eca.mean);
    else if (C.isObject(payload.eca?.raw)) acceptEddy(payload.eca.raw);
    if (kind === "status_update") {
      if (C.isObject(payload.devices)) updateDevices(payload.devices);
      if (C.isObject(payload.streams)) updateStreamHealth(payload.streams);
      if (C.isObject(payload.lidar)) acceptPose(payload.lidar);
      if (C.isObject(payload.pose)) acceptPose(payload.pose);
      if (C.isObject(payload.bridge)) state.bridge = record(payload.bridge, true);
      if (typeof payload.control_enabled === "boolean") state.control = record({enabled:payload.control_enabled, watchdog_stops:payload.watchdog_stops}, true);
      if (C.isObject(payload.control)) state.control = record(payload.control, payload.control.valid !== false, payload.control.reason || "");
    } else if (kind === "status_sensor") {
      if (C.isObject(payload.video)) receiveVideo(payload.video);
      if (C.isObject(payload.lidar) || C.isObject(payload.pose)) acceptPose(payload.lidar || payload.pose);
      if (C.isObject(payload.channel_mean)) acceptEddy(payload);
    } else if (kind === "response" || kind === "job_update" || kind === "task_update" || kind === "event_update") {
      const correlated = settleRequest(payload);
      state.response = record(payload, true); $("response").textContent = JSON.stringify(C.redact(payload), null, 2);
      const devs = {};
      for (const key of Object.keys(C.PHYSICAL_DEVICES)) if (C.isObject(payload[key]) && payload[key].command === "check") devs[key] = payload[key];
      if (Object.keys(devs).length) updateDevices(devs);
      if (C.isObject(payload.result?.task)) acceptTask(payload.result.task);
      if (payload.result?.event) acceptEvents(payload.result.event);
      log("rx", correlated ? "收到关联回包" : "收到共享/未关联回包", payload);
    } else if(kind!=="sensor_data") log("info", "收到其他类型 " + C.clip(kind));
    render();
  }

  function validateVideo(video) {
    return STREAM_IDS.includes(video.camera_id) && video.encoding === "jpeg_base64" && typeof video.data === "string" && video.data.length <= 800000 && /^[A-Za-z0-9+/]+={0,2}$/.test(video.data) && Number.isInteger(video.width) && Number.isInteger(video.height) && video.width > 0 && video.height > 0 && video.width <= 4096 && video.height <= 4096;
  }

  function receiveVideo(video) {
    if (!STREAM_IDS.includes(video.camera_id)) {invalidate("unknown_video_camera_id");return;}
    if (!validateVideo(video)) {invalidate("invalid_video_payload");return;}
    const stream = state.streams[video.camera_id];
    const stampNs = C.stamp(video.stamp_ns);
    if (stampNs && stream.lastSourceStamp) {
      try {if (BigInt(stampNs) < BigInt(stream.lastSourceStamp)) stream.outOfOrder++;} catch (_) {}
    }
    if (stampNs) stream.lastSourceStamp = stampNs;
    const receivedAt = monoNow();
    const meta = C.redact(video); delete meta.data;
    meta.receivedAt = receivedAt; meta.receivedAtIso = isoNow(); meta.connectionId = state.connectionId;
    stream.received++; stream.lastReceived = meta; C.boundedPush(stream.receiveTimes, receivedAt, 180);
    if (C.finite(video.preview_dropped)) stream.serverDropped = video.preview_dropped;
    state.streamHealth[video.camera_id] = record(Object.assign({}, meta, {status:"active"}), true);
    const frame = {meta, data:video.data, connectionId:state.connectionId};
    if (document.hidden) {
      if (stream.hiddenPending) stream.hiddenDropped++;
      stream.hiddenPending = frame; return;
    }
    enqueueDecode(stream, frame);
  }

  function enqueueDecode(stream, frame) {
    if (stream.decoding) {
      if (stream.pending) stream.clientReplaced++;
      stream.pending = frame; return;
    }
    decodeFrame(stream, frame);
  }

  function decodeFrame(stream, frame) {
    stream.decoding = true;
    const image = new Image();
    const finish = () => {
      if (state.streams[stream.id] !== stream) return;
      stream.decoding = false;
      const next = stream.pending; stream.pending = null;
      if (next) decodeFrame(stream, next);
    };
    image.onload = () => {
      if (frame.connectionId !== state.connectionId || state.streams[stream.id] !== stream) {state.lifecycleDiscards++;return;}
      if (image.naturalWidth !== frame.meta.width || image.naturalHeight !== frame.meta.height) {
        stream.dimensionErrors++; log("error", "jpeg_dimensions_mismatch " + stream.id); finish(); return;
      }
      try {
        const canvas = $(stream.id + "-canvas"), context = canvas.getContext("2d", {alpha:false});
        canvas.width = image.naturalWidth; canvas.height = image.naturalHeight; context.drawImage(image, 0, 0);
        const overlay = $(stream.id + "-overlay");overlay.width = image.naturalWidth;overlay.height = image.naturalHeight;
        const displayedAt = monoNow(); stream.displayed++; C.boundedPush(stream.displayTimes, displayedAt, 180);
        stream.lastDisplayed = Object.assign({}, frame.meta, {displayedAt, displayedAtIso:isoNow()});
        drawOverlay(stream.id, stream.lastDisplayed, matchingResult(stream.lastDisplayed));
      } catch (error) {stream.decodeErrors++;log("error", "canvas_commit_failed " + stream.id, {message:error.message});}
      finish();
    };
    image.onerror = () => {
      if (frame.connectionId !== state.connectionId || state.streams[stream.id] !== stream) {state.lifecycleDiscards++;return;}
      stream.decodeErrors++; log("error", "jpeg_decode_failed " + stream.id); finish();
    };
    image.src = "data:image/jpeg;base64," + frame.data;
  }

  function matchingResult(displayed) {
    const key = C.frameKey(displayed, state.robotId); return key ? state.trackingByFrame.get(key) || null : null;
  }

  function points(value) {
    if (!Array.isArray(value)) return [];
    return value.slice(0, 4000).map(item => Array.isArray(item) ? {x:item[0],y:item[1]} : item).filter(item => C.isObject(item) && C.finite(item.x) && C.finite(item.y));
  }

  function drawPolyline(context, values, color, width, dashed) {
    const line = points(values); if (line.length < 2) return;
    context.save(); context.strokeStyle = color;context.lineWidth = width;context.setLineDash(dashed ? [10,7] : []);context.beginPath();context.moveTo(line[0].x,line[0].y);
    for (let index = 1;index < line.length;index++) context.lineTo(line[index].x,line[index].y);
    context.stroke();context.restore();
  }

  function drawOverlay(id, displayed, resultRecord) {
    const canvas = $(id + "-overlay"), context = canvas.getContext("2d");context.clearRect(0,0,canvas.width,canvas.height);
    const stream = state.streams[id];stream.latestResultState = resultRecord ? "matched" : state.tracking ? "unmatched" : "none";
    if (!resultRecord || !resultRecord.valid) return;
    const result = resultRecord.data;
    if (result.coordinate_space !== "image_pixels") {stream.latestResultState = "unsupported_coordinates";return;}
    if ((C.finite(result.image_width) && result.image_width !== displayed.width) || (C.finite(result.image_height) && result.image_height !== displayed.height)) {stream.latestResultState = "dimension_mismatch";return;}
    const status = String(result.status || "").toUpperCase(), predicted = status === "PREDICTED";
    drawPolyline(context, result.left_boundary, predicted ? "#ffd080" : "#72d7ff", 3, predicted);
    drawPolyline(context, result.right_boundary, predicted ? "#ffd080" : "#72d7ff", 3, predicted);
    drawPolyline(context, result.centerline, status === "CONFLICT_2D" ? "#ff7070" : "#70ddb1", 4, predicted);
    const coverage = points(result.reliable_coverage || result.coverage_region);
    if (coverage.length >= 3) {context.save();context.fillStyle="rgba(112,221,177,.15)";context.beginPath();context.moveTo(coverage[0].x,coverage[0].y);coverage.slice(1).forEach(point=>context.lineTo(point.x,point.y));context.closePath();context.fill();context.restore();}
  }

  function fps(samples) {
    const cutoff = monoNow() - 2000, recent = samples.filter(value => value >= cutoff);
    return recent.length > 1 ? (recent.length - 1) * 1000 / (recent[recent.length - 1] - recent[0]) : 0;
  }

  function freshnessFor(key, data) {
    const advertised = state.capabilities?.freshness_ms?.[key]; return C.freshnessMs(key, data, advertised);
  }

  function assess(item, key) {
    const active = currentRecord(item); return C.assessRecord(active || item, monoNow(), freshnessFor(key, item?.data), state.connected && Boolean(active));
  }

  function renderStreams() {
    for (const id of STREAM_IDS) {
      const stream = state.streams[id];
      $(id + "-receive-fps").textContent = fps(stream.receiveTimes).toFixed(1);
      $(id + "-display-fps").textContent = fps(stream.displayTimes).toFixed(1);
      const shown = stream.lastDisplayed, received = stream.lastReceived;
      $(id + "-kind").textContent = C.contentLabel(shown || received);
      const stateNode = $(id + "-state");
      const staleMs = freshnessFor(id, shown || received);
      const stale = !state.connected || !shown || shown.connectionId !== state.connectionId || monoNow() - shown.displayedAt > staleMs;
      if (!shown) {stateNode.className="screen-state";stateNode.textContent="尚未成功显示预览";}
      else if (stale) {stateNode.className="screen-state stale";stateNode.textContent="画面已过期 · 最后显示于 " + shown.displayedAtIso + " · " + ageText(shown.displayedAt) + "前";}
      else {stateNode.className="screen-state hidden";stateNode.textContent="";}
      if (shown) {
        const receivedLine = received ? "最后接收 frame=" + valueText(received.frame_id) + " stamp_ns=" + valueText(C.stamp(received.stamp_ns)) + " 本地=" + received.receivedAtIso : "最后接收 未知";
        const shownLine = "当前显示 frame=" + valueText(shown.frame_id) + " stamp_ns=" + valueText(C.stamp(shown.stamp_ns)) + " 本地=" + shown.displayedAtIso;
        const overlayText = stream.latestResultState === "matched" ? "结果已按同一帧匹配" : stream.latestResultState === "unmatched" ? "结果待匹配/已过期，未叠加" : stream.latestResultState === "unsupported_coordinates" ? "结果坐标非 image_pixels，未叠加" : stream.latestResultState === "dimension_mismatch" ? "结果尺寸不匹配，未叠加" : "无匹配结果";
        $(id + "-meta").textContent = receivedLine + "\n" + shownLine + "\n" + shown.width + "×" + shown.height + " · " + C.contentLabel(shown) + " · source=" + valueText(shown.source_encoding) + "\n收到/显示/解码失败/尺寸错=" + [stream.received,stream.displayed,stream.decodeErrors,stream.dimensionErrors].join("/") + " · 客户端替代=" + stream.clientReplaced + " · 隐藏页未解码=" + stream.hiddenDropped + " · 乱序=" + stream.outOfOrder + " · 板端 preview 覆盖=" + valueText(stream.serverDropped) + "\n" + overlayText + " · 统计窗口=最近2秒，计数范围=本连接";
      } else $(id + "-meta").textContent = "等待成功解码并提交显示；接收 " + stream.received + "，解码失败 " + stream.decodeErrors + "，客户端替代 " + stream.clientReplaced + "。";
    }
  }

  const deviceNodes = Object.create(null);
  function buildDeviceCards() {
    for (const [key, name] of Object.entries(C.PHYSICAL_DEVICES)) {
      const card = document.createElement("div");card.id="device-"+key;card.className="device";
      const title=document.createElement("span");title.className="device-name";title.textContent=name;
      const status=document.createElement("span");status.className="device-status";
      const reason=document.createElement("div");reason.className="device-reason";
      const detail=document.createElement("div");detail.className="device-detail";
      card.append(title,status,reason,detail);$("devices").appendChild(card);deviceNodes[key]={card,status,reason,detail};
    }
  }

  function renderDevices() {
    for (const [key,node] of Object.entries(deviceNodes)) {
      const item=state.devices[key], assessment=assess(item,key), data=item?.data;
      node.card.className="device "+(assessment.state==="active"?"good":["device_error","invalid"].includes(assessment.state)?"bad":["stale","bridge_disconnected"].includes(assessment.state)?"warn":"");
      node.status.textContent=assessment.label;
      node.reason.textContent=data?C.clip(data.reason || item.reason || "未提供最近原因"):"等待板端自检";
      node.detail.textContent=data?"topic: "+valueText(data.topic)+"\ntype: "+valueText(data.message_type)+"\n收到/有效/无效: "+[valueText(data.received),valueText(data.accepted),valueText(data.invalid)].join("/")+"\nstamp_ns: "+valueText(C.stamp(data.stamp_ns))+"\n接收年龄: "+(assessment.ageMs==null?"未知":(assessment.ageMs/1000).toFixed(1)+"s")+" · 新鲜阈值 "+(freshnessFor(key,data)/1000).toFixed(1)+"s": "";
    }
  }

  function renderStreamHealth() {
    const rows=[];
    const keys=new Set([...Object.keys(C.STREAM_NAMES),...Object.keys(state.streamHealth)]);
    for (const key of keys) {
      const item=state.streamHealth[key], a=assess(item,key), d=item?.data || {};
      const tr=document.createElement("tr");
      const name=C.STREAM_NAMES[key]||key;
      const client=state.streams[key];
      const frequency=C.finite(d.actual_hz)?d.actual_hz.toFixed(2)+" Hz":client?(fps(client.receiveTimes).toFixed(1)+" recv / "+fps(client.displayTimes).toFixed(1)+" display FPS"):"未提供";
      const frames=client?("接收 "+client.received+" / 显示 "+client.displayed+" / 客户端替代 "+client.clientReplaced+" / 解码错误 "+(client.decodeErrors+client.dimensionErrors)):("错误 "+valueText(d.errors)+" / 源丢帧 "+valueText(d.source_dropped)+" / 桥丢帧 "+valueText(d.bridge_dropped));
      for (const text of [name,a.label+" · "+(a.ageMs==null?"年龄未知":(a.ageMs/1000).toFixed(1)+"s"),frequency,frames,"标定 "+valueText(d.calibration_status)+" / 同步 "+valueText(d.sync_status)]) {const td=document.createElement("td");td.textContent=text;tr.appendChild(td);}
      rows.push(tr);
    }
    $("stream-health-body").replaceChildren(...rows);
  }

  function renderPose() {
    const item=state.pose, a=assess(item,"lidar_pose"), pose=item?.data, valid=a.fresh&&item?.valid;
    for (const key of ["x","y","z","angle"]) $("pose-"+key).textContent=valid?pose[key].toFixed(4):"—";
    $("pose-status").textContent=valid?"Odometry 活跃 · 未注册到罐体":"位姿 "+a.label;
    $("pose-meta").textContent=pose?"frame: "+valueText(pose.frame_id)+" → "+valueText(pose.child_frame_id)+" · stamp_ns: "+valueText(C.stamp(pose.stamp_ns))+"\n轴方向: "+valueText(pose.axis_convention)+" · 单位: "+valueText(pose.position_unit)+" / "+valueText(pose.angle_unit)+" · 资产坐标注册: "+valueText(pose.asset_registration):"坐标系、轴方向与源时间待回包；未注册到罐体资产坐标。";
    drawTrajectory();
  }

  function setupCanvas(canvas) {
    const rect=canvas.getBoundingClientRect(), ratio=Math.min(devicePixelRatio||1,2), width=Math.max(1,Math.floor(rect.width*ratio)), height=Math.max(1,Math.floor(rect.height*ratio));
    if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;} return {context:canvas.getContext("2d"),width,height,ratio};
  }

  function drawTrajectory() {
    const {context,width,height}=setupCanvas($("trajectory"));context.clearRect(0,0,width,height);context.strokeStyle="#2c4156";context.lineWidth=1;
    for(let i=1;i<5;i++){context.beginPath();context.moveTo(i*width/5,0);context.lineTo(i*width/5,height);context.stroke();context.beginPath();context.moveTo(0,i*height/5);context.lineTo(width,i*height/5);context.stroke();}
    if(state.trajectory.length<2)return;
    const xs=state.trajectory.map(p=>p.x),ys=state.trajectory.map(p=>p.y),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),rangeX=Math.max(maxX-minX,.01),rangeY=Math.max(maxY-minY,.01),pad=18;
    context.strokeStyle="#70ddb1";context.lineWidth=2;context.beginPath();state.trajectory.forEach((p,i)=>{const x=pad+(p.x-minX)/rangeX*(width-2*pad),y=height-pad-(p.y-minY)/rangeY*(height-2*pad);if(i)context.lineTo(x,y);else context.moveTo(x,y);});context.stroke();
    context.fillStyle="#ffd080";const end=state.trajectory[state.trajectory.length-1],ex=pad+(end.x-minX)/rangeX*(width-2*pad),ey=height-pad-(end.y-minY)/rangeY*(height-2*pad);context.beginPath();context.arc(ex,ey,5,0,Math.PI*2);context.fill();
  }

  function drawEddy() {
    const canvas=$("eddy-chart"),{context,width,height}=setupCanvas(canvas);context.clearRect(0,0,width,height);
    if(!state.eddySamples.length)return;
    const channels=Math.min(8,state.eddySamples[0].values.length),colors=["#70ddb1","#8fcaff","#ffd080","#ff9393","#c2a5ff","#74e1e8","#e9a4cb","#d4e37c"];
    for(let c=0;c<channels;c++){
      const values=state.eddySamples.map(s=>s.values[c]).filter(C.finite);if(values.length<2)continue;
      const min=Math.min(...values),max=Math.max(...values),range=Math.max(max-min,1e-9),buckets=Math.min(Math.floor(width),values.length),step=values.length/buckets;
      context.strokeStyle=colors[c];context.lineWidth=1;context.beginPath();
      for(let b=0;b<buckets;b++){const slice=values.slice(Math.floor(b*step),Math.max(Math.floor((b+1)*step),Math.floor(b*step)+1)),lo=Math.min(...slice),hi=Math.max(...slice),x=b/(buckets-1||1)*(width-1),y1=height-4-(lo-min)/range*(height-8),y2=height-4-(hi-min)/range*(height-8);context.moveTo(x,y1);context.lineTo(x,y2);}context.stroke();
    }
    const d=state.eddyMeta?.data;$("eddy-meta").textContent="channel_mean 预览 · 通道 "+channels+" · 样本时间 "+valueText(C.stamp(d?.stamp_ns))+" · 单位 "+valueText(d?.unit)+" · 采用每像素 min/max 保峰聚合；不等同原始波形。";
  }

  function fillDefinitionList(node, entries) {
    const children=[];
    for(const [key,value] of entries){const dt=document.createElement("dt"),dd=document.createElement("dd");dt.textContent=key;dd.textContent=valueText(value);children.push(dt,dd);} node.replaceChildren(...children);
  }

  function renderTracking() {
    const item=state.tracking,a=assess(item,"tracking"),d=item?.data,status=String(d?.status||"").toUpperCase();
    $("tracking-age").textContent=item?"接收年龄 "+(a.ageMs/1000).toFixed(1)+"s · "+a.label:"未收到";
    $("tracking-state").textContent=d?C.trackingLabel(status):"状态未知";
    $("tracking-reason").textContent=d?C.clip(d.reason||d.failure_reason||(a.fresh?"板端未提供简短说明":"结果已过期"),500):"等待板端结果；页面不重新计算巡迹算法。";
    $("tracking-summary").className="tracking-summary "+(["LOST","CONFLICT_2D"].includes(status)?"bad":["PREDICTED","FUSED_CANDIDATE","GEOMETRY_CANDIDATE"].includes(status)?"warn":status==="VISION_2D"?"good":"");
    fillDefinitionList($("tracking-core"),d?[["结果来源",d.source],["后端/模型版本",[d.backend_version,d.model_version].filter(Boolean).join(" / ")||null],["模式",d.mode],["二维有效",d.vision_2d_valid],["三维有效",d.geometry_3d_valid],["身份确认",d.identity_confirmed],["预测",d.predicted],["坐标",d.coordinate_space],["源时间",C.stamp(d.stamp_ns||d.source_stamp_ns)],["匹配帧",d.frame_ref?.frame_id||d.frame_id]]:[]);
    const metricEntries=C.metricEntries(d||{}).map(([k,v])=>[k,v]);
    const diagnostics=C.isObject(d?.diagnostics)?Object.entries(d.diagnostics).slice(0,30):[];
    fillDefinitionList($("tracking-metrics"),metricEntries.concat(diagnostics).length?metricEntries.concat(diagnostics):[["状态","未提供"]]);
  }

  function renderAlerts() {
    const active=[];
    for(const [id,item] of state.alerts){const d=item.data;if(d.active===false||d.recovered===true)continue;active.push({id,data:d,local:false});}
    const storage=state.storage?.data;if(currentRecord(state.storage)&&storage?.mounted===false)active.push({id:"local-storage-unmounted",data:{severity:"critical",message:"采集盘未挂载：任务写盘必须阻断"},local:true});
    const tracking=state.tracking?.data;if(currentRecord(state.tracking)&&["LOST","CONFLICT_2D"].includes(String(tracking?.status).toUpperCase()))active.push({id:"local-tracking-"+tracking.status,data:{severity:tracking.status==="LOST"?"error":"warning",message:"巡迹 "+C.trackingLabel(tracking.status)+"："+C.clip(tracking.reason||"等待后端说明")},local:true});
    const priority={critical:4,error:3,warning:2,info:1};active.sort((a,b)=>(priority[b.data.severity]||0)-(priority[a.data.severity]||0));
    if(!active.length){const p=document.createElement("p");p.className="muted";p.textContent="当前无活动告警（不代表正式验收通过）";$("alert-list").replaceChildren(p);$("top-alert").textContent="";return;}
    const nodes=active.slice(0,30).map(({id,data})=>{const row=document.createElement("div");row.className="alert-item";const severity=document.createElement("span");severity.className="severity "+(data.severity||"warning");severity.textContent=String(data.severity||"warning").toUpperCase();const body=document.createElement("span");body.textContent=C.clip(data.message||data.reason||data.code,500);const button=document.createElement("button");button.className="quiet";button.textContent=state.acknowledged.has(id)?"本页已读":"确认已读";button.disabled=state.acknowledged.has(id);button.onclick=()=>{state.acknowledged.add(id);renderAlerts();};row.append(severity,body,button);return row;});
    $("alert-list").replaceChildren(...nodes);$("top-alert").textContent="最高告警："+C.clip(active[0].data.message||active[0].data.reason||active[0].data.code,300);
  }

  function checkNodes(containerId, checks, emptyText) {
    if(!checks.length){const div=document.createElement("div");div.className="check";div.textContent=emptyText;$(containerId).replaceChildren(div);return;}
    const nodes=checks.slice(0,50).map(item=>{const div=document.createElement("div");const status=String(item.status||item.result||"unknown").toLowerCase();div.className="check "+(["passed","ok","complete","verified"].includes(status)?"good":["failed","error","blocked"].includes(status)?"bad":"warn");div.textContent=C.clip(item.label||item.name||item.id)+"："+C.clip(item.message||item.reason||status);return div;});$(containerId).replaceChildren(...nodes);
  }

  function renderTask() {
    const d=state.task?.data||{},status=String(d.status||"idle").toLowerCase(),label=C.taskLabel(status),index=TASK_STAGES.indexOf(status);
    setBadge("task-state-text",label,["recording","complete","ready"].includes(status)?"good":["failed","incomplete","unknown"].includes(status)?"bad":["idle"].includes(status)?"muted":"warn");
    setBadge("task-badge","采集 "+label,["recording","complete"].includes(status)?"good":["failed","incomplete","unknown"].includes(status)?"bad":status==="idle"?"muted":"warn");
    setBadge("run-badge",d.run_id?"run "+C.clip(d.run_id,48):"run 未开始",d.run_id?"info":"muted");
    const stages=TASK_STAGES.map((stage,i)=>{const div=document.createElement("div");div.className="stage "+(stage===status?"current":(index>=0&&i<index)||(status==="complete"&&i<=index)?"done":"");div.textContent=C.taskLabel(stage);return div;});$("task-stages").replaceChildren(...stages);
    const progress=C.finite(d.progress_percent)?Math.max(0,Math.min(100,d.progress_percent)):index>=0?index/(TASK_STAGES.length-1)*100:0;$("task-progress").style.width=progress+"%";
    $("task-metrics").innerHTML="<span>elapsed <strong>"+escapeHtml(C.finite(d.elapsed_seconds)?d.elapsed_seconds.toFixed(1)+"s":"未知")+"</strong></span><span>写入量 <strong>"+escapeHtml(bytesText(d.bytes_written))+"</strong></span><span>剩余空间 <strong>"+escapeHtml(bytesText(d.free_bytes??state.storage?.data?.free_bytes))+"</strong></span><span>MCAP <strong>"+escapeHtml(valueText(d.mcap_status))+"</strong></span><span>视频 <strong>"+escapeHtml(valueText(d.video_status))+"</strong></span>";
    $("task-message").textContent=C.clip(d.message||d.error||(state.task?"板端未提供状态说明":"等待任务状态。"),500);
    checkNodes("preflight-checks",Array.isArray(d.preflight_checks)?d.preflight_checks:[],"尚无预检结果");
    const finalize=Array.isArray(d.finalize_checks)?d.finalize_checks:[];
    checkNodes("finalize-checks",finalize.length?finalize:[
      {label:"请求 ACK",status:d.request_ack},{label:"writer 关闭",status:d.writer_closed},{label:"MCAP 有效",status:d.mcap_valid},{label:"源文件 SHA",status:d.source_sha_valid},{label:"算法验收",status:d.algorithm_accepted}
    ].filter(item=>item.status!=null),"尚无封存结果");
  }

  function renderEvents() {
    if(!state.events.length){$("events-body").innerHTML='<tr><td colspan="5" class="empty-cell">尚无事件</td></tr>';}
    else {const rows=state.events.map(event=>{const tr=document.createElement("tr"),evidence=Array.isArray(event.evidence)?event.evidence.map(x=>typeof x==="string"?x:x.name+":"+(x.status||"unknown")).join("、"):valueText(event.evidence_status),missing=Array.isArray(event.missing)?"；缺失 "+event.missing.join("、"):"";for(const text of [[event.event_id,event.type].filter(Boolean).join(" / "),C.stamp(event.stamp_ns)||event.source_time||"未提供",event.status||"unknown",evidence+missing,event.error||event.note||"未提供"]){const td=document.createElement("td");td.textContent=C.clip(text,500);tr.appendChild(td);}return tr;});$("events-body").replaceChildren(...rows);}
    if(!state.history.length){$("history-body").innerHTML='<tr><td colspan="5" class="empty-cell">尚无历史任务</td></tr>';}
    else {const rows=state.history.map(run=>{const tr=document.createElement("tr"),handoff=run.download_url?"由板端服务提供下载":run.handoff_command||run.path_hint||"未提供";const cells=[[run.run_id,run.started_at].filter(Boolean).join("\n"),[Array.isArray(run.sensors)?run.sensors.join("、"):"传感器未提供",C.finite(run.duration_seconds)?run.duration_seconds+"s":"时长未提供"].join("\n"),bytesText(run.size_bytes),[run.finalize_status,run.validation_status,run.hardware_field_verified===true?"硬件已验收":run.hardware_field_verified===false?"硬件未验收":"硬件验收未知"].filter(Boolean).join(" / "),handoff];cells.forEach(text=>{const td=document.createElement("td");td.textContent=C.clip(text,600);tr.appendChild(td);});return tr;});$("history-body").replaceChildren(...rows);}
  }

  function escapeHtml(value){return String(value).replace(/[&<>"']/g,char=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"})[char]);}

  function collectionParameters() {
    const output={task_name:$("task-name").value.trim(),asset_id:$("asset-id").value.trim(),seam_id:$("seam-id").value.trim(),operator_note:$("task-note").value.trim(),profile:$("profile").value};
    for(const key of Object.keys(output))if(!output[key])delete output[key];return output;
  }

  function displayedFrameRef() {
    for(const id of ["depth_camera","fisheye"]){const d=state.streams[id].lastDisplayed;if(d&&d.connectionId===state.connectionId)return C.sourceIdentity(d);}return null;
  }

  function controlGate() {
    const move=operationGate("chassis.move"),stop=operationGate("chassis.stop");
    if(!move.allowed||!stop.allowed)return {allowed:false,reason:move.reason||stop.reason};
    const item=currentRecord(state.control),a=C.assessRecord(item,monoNow(),freshnessFor("control",item?.data),state.connected);
    if(!item)return {allowed:false,reason:"板端 control_enabled 未提供或属于旧连接"};
    if(!a.fresh)return {allowed:false,reason:"板端控制能力已过期"};
    if(item.data.enabled!==true)return {allowed:false,reason:"板端控制已关闭"};
    if(!$("enable-control").checked)return {allowed:false,reason:"本页面人工开关未启用"};
    return {allowed:true,reason:"允许发送单次命令"};
  }

  function sendControl(stop) {
    const gate=controlGate();if(!gate.allowed){notice(gate.reason);return;}
    const axes=Array.isArray(state.capabilities?.control?.axes)?state.capabilities.control.axes:[];
    if(!axes.length){notice("板端未声明支持的自由度。");return;}
    const parameters={};
    for(const axis of axes){if(!["vx","vy","wz"].includes(axis))continue;const input=$(axis),text=input.value.trim(),value=Number(text),limit=state.capabilities?.control?.limits?.[axis];if(!text||!Number.isFinite(value)){notice(axis+" 数值无效");return;}if(!C.finite(limit)||limit<=0){notice("板端未提供 "+axis+" 限速");return;}if(Math.abs(value)>limit){notice(axis+" 超出板端限速 ±"+limit);return;}parameters[axis]=stop?0:value;}
    if(!Object.keys(parameters).length){notice("没有可发送的受支持自由度。");return;}
    sendOperation(stop?"chassis.stop":"chassis.move",parameters,{token:$("control-token").value});
  }

  function renderButtons() {
    const status=String(state.task?.data?.status||"idle").toLowerCase();
    const pre=operationGate("collection.preflight"),start=operationGate("collection.start"),stop=operationGate("collection.stop"),query=operationGate("collection.status");
    $("preflight").disabled=!pre.allowed||actionInFlight("collection.preflight")||["starting","recording","stopping","finalizing"].includes(status);
    const busy=["preflighting","starting","recording","stopping","finalizing"].includes(status);
    const replayBusy=["starting","playing","paused","stopping"].includes(window.TankReplay.snapshot().state?.status)||actionInFlight("replay.start");
    const measureBusy=["running","stopping"].includes(state.measurement?.data?.status);
    $("start-recording").disabled=!start.allowed||actionInFlight("collection.start")||busy||replayBusy;
    $("stop-recording").disabled=!stop.allowed||actionInFlight("collection.stop")||(!["starting","recording"].includes(status)&&!measureBusy);
    $("quick-start").disabled=$("start-recording").disabled;
    $("quick-stop").disabled=$("stop-recording").disabled;
    $("measure-thickness").disabled=!operationGate("emat.measure").allowed||actionInFlight("emat.measure")||measureBusy;
    $("measurement-source").disabled=measureBusy;
    const measurement=state.measurement?.data;
    $("thickness-result").textContent=measurement?.status==="completed"&&C.finite(measurement.thickness_mm)?"厚度："+measurement.thickness_mm.toFixed(4)+" mm":"厚度：—";
    $("measurement-message").textContent=measurement?C.clip(measurement.message,500)+(measurement.status==="running"&&C.finite(measurement.progress_percent)?" · "+measurement.progress_percent.toFixed(0)+"%":"")+(measurement.status==="completed"&&measurement.calibrated!==true?" · 使用配置声速，未标定":""):"连接后可启动采集或测厚。";
    $("query-task").disabled=!query.allowed||actionInFlight("collection.status");
    const collectionAvailable=pre.allowed||start.allowed||stop.allowed;setBadge("collection-capability",collectionAvailable?"板端任务能力已声明":"任务能力不可用",collectionAvailable?"good":"warn");
    $("collection-disabled-reason").textContent=collectionAvailable?"按钮随真实任务状态启用；副作用请求超时后先查询，不自动重试。":pre.reason;
    const displayed=operationGate("evidence.save_displayed"),next=operationGate("evidence.save_next"),mark=operationGate("event.mark"),cache=state.capabilities?.snapshot?.displayed_frame_cache===true,frame=displayedFrameRef();
    $("save-displayed").disabled=!displayed.allowed||!cache||!frame||actionInFlight("evidence.save_displayed");
    $("save-next").disabled=!next.allowed||actionInFlight("evidence.save_next");
    $("add-marker").disabled=!mark.allowed||actionInFlight("event.mark");
    const snapshotAvailable=displayed.allowed||next.allowed||mark.allowed;setBadge("snapshot-capability",snapshotAvailable?"板端事件能力已声明":"事件能力不可用",snapshotAvailable?"good":"warn");
    $("snapshot-disabled-reason").textContent=!snapshotAvailable?mark.reason:!cache?"板端未声明显示帧缓存：只能请求下一处理帧或添加标记。":"“当前显示帧”发送精确 frame_ref；accepted 仅表示入队，complete/hash 才表示完整。";
    const history=operationGate("history.list");$("query-history").disabled=!history.allowed||actionInFlight("history.list");$("history-reason").textContent=history.allowed?"列表来自板端配置的数据根/登记索引；封存源 run 只读。":history.reason;
    const control=controlGate();$("move").disabled=!control.allowed||actionInFlight("chassis.move");$("stop").disabled=!control.allowed||actionInFlight("chassis.stop");$("control-status").textContent=control.reason+"；零速度不是硬件急停。";
    const axes=Array.isArray(state.capabilities?.control?.axes)?state.capabilities.control.axes:[];for(const axis of ["vx","vy","wz"]){$(axis+"-field").classList.toggle("hidden",!axes.includes(axis));const limit=state.capabilities?.control?.limits?.[axis];if(C.finite(limit)){$(axis).min=-limit;$(axis).max=limit;}}
    window.TankReplay.render(state.connected);
  }

  function renderTop() {
    $("robot-badge").textContent="机器人 "+state.robotId+(state.robotSource==="default"?"（默认）":"");
    setBadge("mode-badge","模式 "+(state.mode?C.clip(state.mode):"未知"),state.mode==="live"?"good":["mock","simulation","replay"].includes(String(state.mode).toLowerCase())?"warn":"muted");
    const storage=state.storage?.data,a=assess(state.storage,"storage");
    if(!storage)setBadge("ssd-badge","SSD 未知","muted");else if(storage.mounted===false)setBadge("ssd-badge","SSD 未挂载","bad");else if(!a.fresh)setBadge("ssd-badge","SSD 状态过期","warn");else setBadge("ssd-badge","SSD "+bytesText(storage.free_bytes)+" 可用","good");
  }

  function render() {
    for(const key of ["rx","tx","invalid","foreign"])$(key+"-count").textContent=state[key];
    $("last-received").textContent=state.lastPacket==null?"尚无回包":"距收到任意回包 "+ageText(state.lastPacket);
    renderTop();renderStreams();renderDevices();renderStreamHealth();renderPose();renderTracking();renderTask();renderEvents();renderAlerts();drawEddy();renderButtons();
    $("bridge-meta").textContent=state.bridge?.data?"桥接 command_dropped="+valueText(state.bridge.data.command_dropped)+" · transport_dropped="+valueText(state.bridge.data.transport_dropped)+"（含无连接期间；不等同页面丢帧）":"等待桥接统计";
  }

  function snapshot() {
    const streamSnapshot=Object.fromEntries(STREAM_IDS.map(id=>{const s=state.streams[id];return [id,{counts_scope:"current_connection",received:s.received,displayed:s.displayed,decode_errors:s.decodeErrors,dimension_errors:s.dimensionErrors,client_replaced_before_decode:s.clientReplaced,hidden_not_decoded:s.hiddenDropped,out_of_order:s.outOfOrder,server_preview_dropped:s.serverDropped,receive_fps_2s:fps(s.receiveTimes),display_fps_2s:fps(s.displayTimes),last_received_frame:s.lastReceived?C.redact(s.lastReceived):null,last_displayed_frame:s.lastDisplayed?C.redact(s.lastDisplayed):null,overlay_match:s.latestResultState}];}));
    return C.redact({schema_version:"tank_inspection_console_delivery_v1",exported_at:isoNow(),endpoint:state.endpoint,robot_id:state.robotId,robot_identity_source:state.robotSource,mode:state.mode,connected:state.connected,connection_count:state.connectionCount,counts_scope:"page_lifetime",counts:{received:state.rx,sent:state.tx,invalid:state.invalid,foreign:state.foreign,lifecycle_old_callback_discarded:state.lifecycleDiscards},capabilities:state.capabilities,task:state.task?.data,measurement:state.measurement?.data,storage:state.storage?.data,devices:Object.fromEntries(Object.entries(state.devices).map(([k,v])=>[k,v.data])),streams:streamSnapshot,pose:state.pose?.data,tracking:state.tracking?.data,events:state.events,history:state.history,request_history:state.requestHistory.slice(-50),last_response:state.response?.data,legacy_protocol_compatible:true,response_correlation:state.capabilities?"request_id_when_provided; uncorrelated otherwise":"unavailable_no_request_id",console_software_verified:false,collection_workflow_verified:false,snapshot_workflow_verified:false,hardware_field_verified:false,production_status_source:"formal_project_acceptance_not_browser",logs:state.logs});
  }

  function exportSnapshot() {
    const url=URL.createObjectURL(new Blob([JSON.stringify(snapshot(),null,2)],{type:"application/json"}));const a=document.createElement("a");a.href=url;a.download="tank-console-"+isoNow().replace(/[:.]/g,"-")+".json";a.click();window.setTimeout(()=>URL.revokeObjectURL(url),1000);
  }

  function startRenderLoop() {if(renderTimer!==null)return;renderTimer=window.setInterval(render,document.hidden?1000:250);}
  function stopRenderLoop(){if(renderTimer!==null){clearInterval(renderTimer);renderTimer=null;}}
  function restartRenderLoop(){stopRenderLoop();startRenderLoop();}

  function bindUi() {
    document.querySelectorAll("nav button[data-view]").forEach(button=>button.onclick=()=>{document.querySelectorAll("nav button").forEach(x=>x.classList.toggle("active",x===button));document.querySelectorAll(".view").forEach(x=>x.classList.toggle("active",x.id==="view-"+button.dataset.view));render();});
    $("connect").onclick=connect;$("disconnect").onclick=disconnect;
    $("check").onclick=()=>sendLegacy("check",{});
    $("start-stream").onclick=()=>sendLegacy("command",{camera:{command:"start_stream"}});
    $("stop-stream").onclick=()=>sendLegacy("command",{camera:{command:"stop_stream"}});
    $("reconnect").onchange=()=>{if(!$("reconnect").checked&&!state.connected&&retryTimer)disconnect();};
    $("remember-endpoint").onchange=()=>{try{if(!$("remember-endpoint").checked)localStorage.removeItem("tank_console_endpoint");}catch(_){}};
    $("preflight").onclick=()=>sendOperation("collection.preflight",collectionParameters());
    $("start-recording").onclick=()=>sendOperation("collection.start",collectionParameters());
    $("stop-recording").onclick=()=>sendOperation("collection.stop",{run_id:state.task?.data?.run_id});
    $("quick-start").onclick=$("start-recording").onclick;
    $("quick-stop").onclick=$("stop-recording").onclick;
    $("measure-thickness").onclick=()=>sendOperation("emat.measure",{source:$("measurement-source").value});
    $("query-task").onclick=()=>sendOperation("collection.status",{});
    $("save-displayed").onclick=()=>{const frame_ref=displayedFrameRef();if(frame_ref)sendOperation("evidence.save_displayed",{type:$("event-type").value,note:$("event-note").value.trim(),frame_ref});};
    $("save-next").onclick=()=>sendOperation("evidence.save_next",{type:$("event-type").value,note:$("event-note").value.trim()});
    $("add-marker").onclick=()=>sendOperation("event.mark",{type:$("event-type").value,note:$("event-note").value.trim(),displayed_frame_ref:displayedFrameRef()});
    $("query-history").onclick=()=>sendOperation("history.list",{page_size:50});
    $("enable-control").onchange=renderButtons;$("move").onclick=()=>sendControl(false);$("stop").onclick=()=>sendControl(true);
    $("clear-log").onclick=()=>{state.logs=[];$("log").replaceChildren();};$("export").onclick=exportSnapshot;
    document.querySelectorAll("[data-overlay]").forEach(button=>button.onclick=()=>{const id=button.dataset.overlay,stream=state.streams[id];stream.overlayVisible=!stream.overlayVisible;$(id+"-screen").classList.toggle("raw-only",!stream.overlayVisible);button.textContent=stream.overlayVisible?"隐藏叠加":"显示叠加";});
    document.querySelectorAll("[data-fullscreen]").forEach(button=>button.onclick=()=>{const node=$(button.dataset.fullscreen+"-screen");if(node.requestFullscreen)node.requestFullscreen();});
    window.addEventListener("resize",()=>{drawTrajectory();drawEddy();});
    document.addEventListener("visibilitychange",()=>{restartRenderLoop();if(!document.hidden){for(const id of STREAM_IDS){const stream=state.streams[id];if(stream.hiddenPending){const frame=stream.hiddenPending;stream.hiddenPending=null;enqueueDecode(stream,frame);}}render();}});
    window.addEventListener("pagehide",()=>{pageReconnectWanted=state.desired;closeSocket(false,true);state.desired=false;stopRenderLoop();});
    window.addEventListener("pageshow",event=>{startRenderLoop();if(event.persisted&&pageReconnectWanted&&$("reconnect").checked&&!state.connected){state.desired=true;openSocket();}render();});
  }

  function initializeEndpoint() {
    const query=new URLSearchParams(location.search),remembered=(()=>{try{return localStorage.getItem("tank_console_endpoint");}catch(_){return null;}})();
    if(query.has("ws"))$("endpoint").value=query.get("ws");
    else if(remembered){$("endpoint").value=remembered;$("remember-endpoint").checked=true;}
    else if(location.hostname){const host=location.hostname.includes(":")?"["+location.hostname+"]":location.hostname;$("endpoint").value=(location.protocol==="https:"?"wss://":"ws://")+host+":9001";}
  }

  buildDeviceCards();bindUi();window.TankReplay.init({send:sendOperation,gate:operationGate,inFlight:actionInFlight,taskStatus:()=>state.task?.data?.status});initializeEndpoint();updateConnectionUi("未连接","warn");startRenderLoop();render();
  if(location.protocol!=="file:")connect();
  window.tankConsole=Object.freeze({snapshot,version:"TANK-CONSOLE-DELIVERY-V1"});
})();
