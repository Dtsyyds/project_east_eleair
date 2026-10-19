/* Shared, dependency-free state helpers for the tank inspection console. */
"use strict";
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.TankConsoleCore = Object.freeze(api);
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  const PHYSICAL_DEVICES = Object.freeze({
    fisheye: "鱼眼相机",
    mindvision: "Mindvision 工业相机",
    depth_camera: "D405 相机",
    eddy_current: "涡流设备",
    emat: "电磁超声",
    chassis: "底盘",
    lidar: "激光雷达"
  });

  const STREAM_NAMES = Object.freeze({
    mindvision: "工业相机预览",
    fisheye: "鱼眼预览",
    depth_camera: "D405 预览",
    lidar_pose: "雷达位姿",
    eddy_raw: "涡流原始数据",
    eddy_mean: "涡流通道均值"
  });

  const DEFAULT_FRESHNESS_MS = Object.freeze({
    fisheye: 3500,
    mindvision: 3500,
    depth_camera: 3500,
    eddy_current: 2500,
    emat: 5000,
    chassis: 2500,
    lidar: 2500,
    lidar_pose: 2500,
    tracking: 2500,
    control: 2500,
    task: 10000,
    storage: 15000
  });

  const TRACKING_LABELS = Object.freeze({
    FUSED_CANDIDATE: "融合候选",
    VISION_2D: "二维视觉",
    GEOMETRY_CANDIDATE: "几何候选",
    CONFLICT_2D: "二维冲突",
    PREDICTED: "预测",
    LOST: "巡迹丢失"
  });

  const TASK_LABELS = Object.freeze({
    idle: "未开始",
    preflighting: "预检中",
    ready: "就绪",
    starting: "启动中",
    recording: "采集中",
    stopping: "停止中",
    finalizing: "校验/封存中",
    complete: "完整完成",
    incomplete: "未完整",
    failed: "失败",
    unknown: "结果未知"
  });

  const isObject = value => value !== null && typeof value === "object" && !Array.isArray(value);
  const finite = value => typeof value === "number" && Number.isFinite(value);
  const clip = (value, max = 256, fallback = "未提供") => String(value == null || value === "" ? fallback : value).slice(0, max);
  const stamp = value => typeof value === "string" && /^-?\d{1,24}$/.test(value) ? value : null;

  function redact(value) {
    let budget = 4096;
    function copy(item, depth) {
      if (--budget < 0 || depth > 6) return "[TRUNCATED]";
      if (typeof item === "string") return clip(item, 768);
      if (Array.isArray(item)) return item.slice(0, 96).map(v => copy(v, depth + 1));
      if (!isObject(item)) return item;
      const out = Object.create(null);
      for (const key of Object.keys(item).slice(0, 48)) {
        if (/token|password|authorization|cookie/i.test(key)) out[key] = "[REDACTED]";
        else if (/^(data|jpeg|image_base64|depth_raw)$/i.test(key)) out[key] = "[BODY OMITTED]";
        else out[key] = copy(item[key], depth + 1);
        if (budget <= 0) break;
      }
      return out;
    }
    return copy(value, 0);
  }

  function makeRecord(data, receivedAt, valid = true, reason = "") {
    return {data:redact(data), receivedAt, valid:Boolean(valid), reason:clip(reason, 256, "")};
  }

  function configuredState(data) {
    if (!isObject(data)) return "unknown";
    if (data.configured === false) return "not_configured";
    if (data.enabled === false) return "disabled";
    return "configured";
  }

  function freshnessMs(key, data, advertised) {
    if (finite(advertised) && advertised >= 250 && advertised <= 300000) return advertised;
    if (isObject(data)) {
      if (finite(data.freshness_ms) && data.freshness_ms >= 250 && data.freshness_ms <= 300000) return data.freshness_ms;
      if (finite(data.expected_hz) && data.expected_hz > 0 && data.expected_hz <= 10000) return Math.min(300000, Math.max(750, 3 * 1000 / data.expected_hz));
      if (finite(data.expected_period_ms) && data.expected_period_ms > 0) return Math.min(300000, Math.max(750, 3 * data.expected_period_ms));
    }
    return DEFAULT_FRESHNESS_MS[key] || 5000;
  }

  function assessRecord(record, nowMs, thresholdMs, connected = true) {
    if (!record) return {state:"unknown", label:"未知", fresh:false, ageMs:null};
    const cfg = configuredState(record.data);
    const ageMs = Math.max(0, nowMs - record.receivedAt);
    if (cfg === "not_configured") return {state:cfg, label:"未配置", fresh:true, ageMs};
    if (cfg === "disabled") return {state:cfg, label:"已禁用", fresh:true, ageMs};
    if (!connected) return {state:"bridge_disconnected", label:"桥接断开", fresh:false, ageMs};
    if (!record.valid) return {state:"invalid", label:"数据无效", fresh:ageMs <= thresholdMs, ageMs};
    if (ageMs > thresholdMs) return {state:"stale", label:"已过期", fresh:false, ageMs};
    const status = record.data && record.data.status;
    if (["failed", "error", "fault"].includes(status)) return {state:"device_error", label:"设备错误", fresh:true, ageMs};
    if (["verified", "active", "ok", "running"].includes(status)) return {state:"active", label:"活跃", fresh:true, ageMs};
    return {state:"unknown", label:"状态未知", fresh:true, ageMs};
  }

  function contentLabel(video) {
    if (!isObject(video)) return "内容类型未提供";
    const value = String(video.content_type || video.image_type || video.semantic || video.source_encoding || "").toLowerCase();
    if (/fusion|fused|overlay/.test(value)) return "融合叠加";
    if (/depth.*color|color.*depth|depth_colormap|depth_visual/.test(value)) return "深度伪彩";
    if (/infra.?1|infrared.?1|ir.?1/.test(value)) return "Infra1";
    if (/rgb|bgr|color|colour/.test(value)) return "彩色";
    if (/mono|gray|grey/.test(value)) return "灰度";
    return "内容类型未提供";
  }

  function sourceIdentity(value) {
    if (!isObject(value)) return null;
    const ref = isObject(value.frame_ref) ? value.frame_ref : isObject(value.source_ref) ? value.source_ref : value;
    const robot = ref.robot_id || value.robot_id || "";
    const source = ref.source_id || ref.source || ref.camera_id || value.camera_id || "";
    const stream = ref.stream_id || ref.stream || ref.topic || "";
    const frame = ref.frame_id || "";
    const timestamp = stamp(ref.stamp_ns || ref.source_stamp_ns || value.stamp_ns);
    if (!source || (!frame && !timestamp)) return null;
    return {robot:String(robot), source:String(source), stream:String(stream), frame:String(frame), stamp_ns:timestamp || ""};
  }

  function frameKey(value, fallbackRobot = "") {
    const identity = sourceIdentity(value);
    if (!identity) return null;
    return [identity.robot || fallbackRobot, identity.source, identity.stream, identity.frame, identity.stamp_ns].map(part => encodeURIComponent(part)).join("|");
  }

  function sameFrame(left, right, fallbackRobot = "") {
    const a = frameKey(left, fallbackRobot), b = frameKey(right, fallbackRobot);
    return Boolean(a && b && a === b);
  }

  function trackingLabel(status) {
    return TRACKING_LABELS[String(status || "").toUpperCase()] || "状态未知";
  }

  function taskLabel(status) {
    return TASK_LABELS[String(status || "").toLowerCase()] || "未知状态";
  }

  function operationSpec(capabilities, action) {
    if (!isObject(capabilities) || !isObject(capabilities.operations)) return null;
    const value = capabilities.operations[action];
    if (value === true) return {supported:true, msg_type:"command"};
    if (!isObject(value) || value.supported === false) return null;
    const msgType = value.msg_type || "command";
    if (!["command", "check"].includes(msgType)) return null;
    return Object.assign({supported:true, msg_type:msgType}, redact(value));
  }

  function operationGate(capabilities, action, connected) {
    if (!connected) return {allowed:false, reason:"桥接未连接"};
    if (!isObject(capabilities)) return {allowed:false, reason:"板端未声明能力，按旧协议禁用"};
    const spec = operationSpec(capabilities, action);
    if (!spec) return {allowed:false, reason:"板端未声明支持此操作"};
    const permissions = capabilities.permissions;
    if (isObject(permissions) && permissions[action] === false) return {allowed:false, reason:"当前会话无权限"};
    return {allowed:true, reason:"", spec};
  }

  function safeEndpoint(value, pageProtocol) {
    const url = new URL(String(value).trim());
    if (!["ws:", "wss:"].includes(url.protocol) || url.username || url.password || url.hash) throw new Error("请输入 ws:// 或 wss:// 地址，不携带口令或片段。");
    if (pageProtocol === "https:" && url.protocol !== "wss:") throw new Error("HTTPS 页面只能连接 wss:// 地址。");
    return url.href;
  }

  function boundedPush(items, value, limit) {
    items.push(value);
    if (items.length > limit) items.splice(0, items.length - limit);
    return items;
  }

  function metricEntries(result) {
    if (!isObject(result)) return [];
    const metrics = isObject(result.metrics) ? result.metrics : {};
    const allowed = [
      "image_depth_delta_ms", "depth_valid_raw", "depth_valid_registered", "depth_valid_filtered",
      "background_fit_quality", "candidate_count", "coverage", "joint_inlier", "residual",
      "confidence", "processing_ms"
    ];
    return allowed.filter(key => Object.hasOwn(metrics, key)).map(key => [key, metrics[key]]);
  }

  return {
    PHYSICAL_DEVICES, STREAM_NAMES, DEFAULT_FRESHNESS_MS, TRACKING_LABELS, TASK_LABELS,
    isObject, finite, clip, stamp, redact, makeRecord, configuredState, freshnessMs,
    assessRecord, contentLabel, sourceIdentity, frameKey, sameFrame, trackingLabel,
    taskLabel, operationSpec, operationGate, safeEndpoint, boundedPush, metricEntries
  };
});
