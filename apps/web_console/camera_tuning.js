"use strict";
(() => {
  const fields = ["auto_exposure", "exposure_time_us", "analog_gain", "ae_target", "white_balance",
                  "rgb_gain_r", "rgb_gain_g", "rgb_gain_b", "saturation", "gamma", "contrast"];
  const $ = id => document.getElementById(id);
  let api, current = null, pending = null, lastKey = null, dirty = false, needsRead = true;
  const message = text => {$("camera-tuning-message").textContent = text;};
  const busyTask = status => ["preflighting", "starting", "recording", "stopping", "finalizing"].includes(status);

  function fill(data) {
    for (const field of fields) {
      const input = $("mv-"+field), scale = field === "exposure_time_us" ? 1000 : 1;
      input.value = typeof data.values[field] === "number" ? data.values[field] / scale : String(data.values[field]);
      if (data.ranges[field]) {
        const [low, high, step] = data.ranges[field];
        input.min = low / scale; input.max = high / scale;
        // Exposure is quantized by the SDK; the returned value is authoritative.
        input.step = field === "exposure_time_us" ? "any" : step;
        input.title = `范围 ${low / scale}–${high / scale}`;
      }
    }
    $("mv-white_balance").querySelector('option[value="auto"]').disabled = !data.supports_auto_white_balance;
    dirty = false;
  }

  function send(action) {
    const parameters = {};
    if (action !== "camera.settings.get") {
      if (!current || needsRead) {message("请先读取当前参数。");return;}
      const values = {};
      for (const field of fields) {
        const input = $("mv-"+field);
        if (field === "auto_exposure") values[field] = input.value === "true";
        else if (field === "white_balance") values[field] = input.value;
        else {
          if (input.value.trim() === "" || !Number.isFinite(Number(input.value)) || !input.reportValidity()) {
            message("请输入范围内的有效参数。");return;
          }
          values[field] = Number(input.value) * (field === "exposure_time_us" ? 1000 : 1);
        }
      }
      Object.assign(parameters, {values, serial_number:current.serial_number, expected_revision:current.settings_revision});
    }
    const id = api.send(action, parameters);
    if (id) {pending = id;message(action === "camera.settings.get" ? "正在读取相机实际参数…" : "正在应用并核对相机参数…");}
  }

  function receive(packet) {
    const payload = packet.payload;
    const ownRequest = pending;
    if (payload.request_id === pending && ["failed", "completed"].includes(payload.status)) {
      pending = null;
      if (payload.status === "failed") {needsRead = true;message(payload.error || "参数操作失败，请重新读取当前参数。");}
    }
    const data = payload.camera_settings;
    if (!data || data.status !== "completed" || !data.values) return;
    const key = data.service_id + ":" + data.request_id;
    if (key === lastKey) return;
    lastKey = key;
    const own = data.request_id === ownRequest;
    current = data; needsRead = false;
    $("camera-tuning-info").textContent = `相机 ${data.serial_number} · ${data.width}×${data.height} · 发布 ${data.publish_rate_hz} Hz`;
    if (!dirty || own) {
      fill(data);
      message(data.fault || (data.saved ? "当前参数已保存，后续任务及重启自动沿用。" : "当前参数尚未保存为后续任务默认值。"));
    } else {
      needsRead = true;
      message("相机参数已更新；当前草稿保留，请读取最新参数后重新调整。");
    }
  }

  function render(connected, taskStatus, replayStatus) {
    if (!api) return;
    const busy = pending !== null || api.inFlight("camera.settings.get");
    const locked = busyTask(taskStatus) || ["starting", "playing", "paused", "stopping"].includes(replayStatus);
    const editable = connected && !busy && !locked && current !== null && !needsRead;
    $("camera-tuning-read").disabled = !api.gate("camera.settings.get").allowed || busy;
    $("camera-tuning-apply").disabled = !editable || !api.gate("camera.settings.apply").allowed;
    $("camera-tuning-save").disabled = !editable || !api.gate("camera.settings.save").allowed;
    for (const field of fields) $("mv-"+field).disabled = !editable;
    if (editable) {
      const auto = $("mv-auto_exposure").value === "true", wb = $("mv-white_balance").value === "auto";
      $("mv-exposure_time_us").disabled = auto; $("mv-analog_gain").disabled = auto;
      $("mv-ae_target").disabled = !auto;
      for (const field of ["rgb_gain_r", "rgb_gain_g", "rgb_gain_b"]) $("mv-"+field).disabled = wb;
    }
    if (locked) message("采集或回放期间参数已锁定，请先停止任务。");
    else if (!connected) message("连接已断开，重新连接后读取当前参数。");
  }

  function init(options) {
    api = options;
    $("camera-tuning-toggle").onclick = () => {
      const panel = $("camera-tuning-panel"), open = panel.classList.contains("hidden");
      panel.classList.toggle("hidden", !open);
      $("camera-tuning-toggle").setAttribute("aria-expanded", String(open));
      if (open && api.gate("camera.settings.get").allowed) send("camera.settings.get");
    };
    $("camera-tuning-read").onclick = () => send("camera.settings.get");
    $("camera-tuning-apply").onclick = () => send("camera.settings.apply");
    $("camera-tuning-save").onclick = () => send("camera.settings.save");
    for (const field of fields) $("mv-"+field).oninput = () => {dirty = true;message("参数已编辑，尚未应用。可先应用预览，再保存沿用。");};
  }
  function reset() {current = null; pending = null; lastKey = null; needsRead = true; dirty = false;}
  function timeout(id) {if (pending === id) {pending = null;needsRead = true;message("响应超时，结果未知；请读取当前参数，勿重复提交。");}}
  window.CameraTuning = Object.freeze({init, receive, render, reset, timeout, snapshot:() => current});
})();
