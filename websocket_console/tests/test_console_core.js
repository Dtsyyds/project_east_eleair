"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const C = require("../console_core.js");

test("partial records have independent freshness", () => {
  const camera = C.makeRecord({status:"verified", expected_hz:10}, 1000);
  const pose = C.makeRecord({status:"active", expected_hz:20}, 1000);
  const updatedCamera = C.makeRecord({status:"verified", expected_hz:10}, 5000);
  assert.equal(C.assessRecord(updatedCamera, 5100, C.freshnessMs("fisheye", updatedCamera.data), true).state, "active");
  assert.equal(C.assessRecord(pose, 5100, C.freshnessMs("lidar_pose", pose.data), true).state, "stale");
  assert.equal(C.assessRecord(camera, 5100, C.freshnessMs("fisheye", camera.data), true).state, "stale");
});

test("configured, disabled, stale, error and disconnected are distinct", () => {
  assert.equal(C.assessRecord(C.makeRecord({configured:false}, 0), 9999, 1000, true).state, "not_configured");
  assert.equal(C.assessRecord(C.makeRecord({enabled:false}, 0), 9999, 1000, true).state, "disabled");
  assert.equal(C.assessRecord(C.makeRecord({status:"failed"}, 1000), 1100, 1000, true).state, "device_error");
  assert.equal(C.assessRecord(C.makeRecord({status:"active"}, 1000), 3000, 1000, true).state, "stale");
  assert.equal(C.assessRecord(C.makeRecord({status:"active"}, 1000), 1100, 1000, false).state, "bridge_disconnected");
});

test("frame matching requires source, stream, frame and decimal nanoseconds", () => {
  const video = {robot_id:"r1", camera_id:"depth_camera", stream_id:"preview", frame_id:"f7", stamp_ns:"18446744073709551615"};
  const exact = {frame_ref:{robot_id:"r1", source_id:"depth_camera", stream_id:"preview", frame_id:"f7", stamp_ns:"18446744073709551615"}};
  const newer = {frame_ref:{robot_id:"r1", source_id:"depth_camera", stream_id:"preview", frame_id:"f8", stamp_ns:"18446744073709551616"}};
  assert.equal(C.sameFrame(video, exact), true);
  assert.equal(C.sameFrame(video, newer), false);
  assert.equal(C.stamp(18446744073709551615), null, "Number timestamps are rejected to avoid precision loss");
});

test("video semantics come from advertised content, never camera id", () => {
  assert.equal(C.contentLabel({camera_id:"depth_camera", content_type:"rgb8"}), "彩色");
  assert.equal(C.contentLabel({camera_id:"depth_camera", content_type:"infra1"}), "Infra1");
  assert.equal(C.contentLabel({camera_id:"depth_camera", content_type:"depth_colormap"}), "深度伪彩");
  assert.equal(C.contentLabel({camera_id:"depth_camera"}), "内容类型未提供");
});

test("new operations are disabled until explicitly advertised", () => {
  assert.deepEqual(C.operationGate(null, "collection.start", true), {allowed:false, reason:"板端未声明能力，按旧协议禁用"});
  assert.equal(C.operationGate({operations:{}}, "collection.start", true).allowed, false);
  assert.equal(C.operationGate({operations:{"collection.start":true}}, "collection.start", true).allowed, true);
  assert.equal(C.operationGate({operations:{"collection.start":{supported:true}},permissions:{"collection.start":false}}, "collection.start", true).reason, "当前会话无权限");
  assert.equal(C.operationGate({operations:{"collection.start":{msg_type:"remote_shell"}}}, "collection.start", true).allowed, false);
});

test("redaction removes credentials and binary bodies", () => {
  const value = C.redact({token:"secret", nested:{password:"pw"}, data:"jpeg", safe:"ok"});
  assert.equal(value.token, "[REDACTED]");
  assert.equal(value.nested.password, "[REDACTED]");
  assert.equal(value.data, "[BODY OMITTED]");
  assert.equal(value.safe, "ok");
});

test("HTTPS rejects ws and endpoints reject embedded credentials", () => {
  assert.throws(() => C.safeEndpoint("ws://robot:9001", "https:"), /wss/);
  assert.throws(() => C.safeEndpoint("wss://user:pass@robot/ws", "https:"), /不携带口令/);
  assert.equal(C.safeEndpoint("wss://robot.example/ws", "https:"), "wss://robot.example/ws");
});

test("algorithm states remain candidates and failures, not confirmed welds", () => {
  assert.equal(C.trackingLabel("FUSED_CANDIDATE"), "融合候选");
  assert.equal(C.trackingLabel("GEOMETRY_CANDIDATE"), "几何候选");
  assert.equal(C.trackingLabel("LOST"), "巡迹丢失");
});
