//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "driver_node.h"
#include "lddc.h"

#include <chrono>
#include <string>

namespace livox_ros {

DriverNode& DriverNode::GetNode() noexcept {
  return *this;
}

DriverNode::~DriverNode() {
  if (lddc_ptr_ && lddc_ptr_->lds_) {
    lddc_ptr_->lds_->RequestExit();
  }
  exit_signal_.set_value();
  if (pointclouddata_poll_thread_ && pointclouddata_poll_thread_->joinable()) {
    pointclouddata_poll_thread_->join();
  }
  if (imudata_poll_thread_ && imudata_poll_thread_->joinable()) {
    imudata_poll_thread_->join();
  }
}

void DriverNode::PublishContinuityDiagnostics()
{
  using diagnostic_msgs::msg::DiagnosticArray;
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;
  const int64_t now_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const uint64_t lidar_count = lddc_ptr_ ? lddc_ptr_->LidarPublishCount() : 0;
  const uint64_t imu_count = lddc_ptr_ ? lddc_ptr_->ImuPublishCount() : 0;
  const int64_t lidar_last = lddc_ptr_ ? lddc_ptr_->LastLidarPublishSteadyNs() : 0;
  const int64_t imu_last = lddc_ptr_ ? lddc_ptr_->LastImuPublishSteadyNs() : 0;
  const double uptime_sec = (now_steady_ns - diagnostics_started_steady_ns_) / 1e9;
  const double lidar_age_sec = lidar_last > 0 ? (now_steady_ns - lidar_last) / 1e9 : uptime_sec;
  const double imu_age_sec = imu_last > 0 ? (now_steady_ns - imu_last) / 1e9 : uptime_sec;
  const bool startup_grace = uptime_sec < 8.0;
  const bool source_stale = !startup_grace && (lidar_age_sec > 2.0 || imu_age_sec > 2.0);

  DiagnosticArray array;
  array.header.stamp = now();
  DiagnosticStatus status;
  status.name = "livox_ros_driver2/publisher";
  status.hardware_id = "MID-360";
  status.level = source_stale ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
  status.message = source_stale ? "SOURCE_STOPPED" :
    (startup_grace ? "STARTING" : "SOURCE_STREAMING");
  const auto add = [&status](const std::string & key, const std::string & value) {
      KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  add("classification", status.message);
  add("lidar_publish_count", std::to_string(lidar_count));
  add("imu_publish_count", std::to_string(imu_count));
  add("lidar_last_publish_age_sec", std::to_string(lidar_age_sec));
  add("imu_last_publish_age_sec", std::to_string(imu_age_sec));
  add("publisher_heartbeat_steady_ns", std::to_string(now_steady_ns));
  add("timestamp_clock", "steady_clock_for_age_only");
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
}

} // namespace livox_ros





