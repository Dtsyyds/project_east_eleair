// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef REALSENSE2_CAMERA_HARDWARE_CLOCK_MAPPER_H
#define REALSENSE2_CAMERA_HARDWARE_CLOCK_MAPPER_H

#include <cmath>
#include <cstdint>
#include <string>

namespace realsense2_camera
{

struct HardwareClockMapping
{
  int64_t stamp_ns{0};
  bool valid{false};
  bool epoch_rebuilt{false};
  const char * reason{"uninitialized"};
  uint64_t epoch{0};
};

struct DirectEpochValidation
{
  bool valid{false};
  const char * reason{"uninitialized"};
  int64_t delta_ns{0};
};

// GLOBAL_TIME and SYSTEM_TIME are already expressed in the Unix/OS epoch.
// They must not be re-anchored.  This validator instead makes a bad mapping
// observable and lets the caller mark that frame invalid.
class DirectEpochTimestampValidator
{
public:
  DirectEpochValidation validate(int64_t stamp_ns)
  {
    if (stamp_ns <= 0) {
      return {false, "direct_epoch_time_out_of_range", 0};
    }
    if (last_stamp_ns_ > 0 && stamp_ns <= last_stamp_ns_) {
      return {false, stamp_ns == last_stamp_ns_ ?
        "direct_epoch_duplicate" : "direct_epoch_regression", stamp_ns - last_stamp_ns_};
    }
    const int64_t delta = last_stamp_ns_ > 0 ? stamp_ns - last_stamp_ns_ : 0;
    last_stamp_ns_ = stamp_ns;
    return {true, "direct_epoch_valid", delta};
  }

private:
  int64_t last_stamp_ns_{0};
};

// Maps a device-relative millisecond clock onto the ROS system epoch.  A real
// hardware clock reset creates a new mapping epoch; it is not hidden by
// clamping the output or manufacturing a last+1 timestamp.
class HardwareClockMapper
{
public:
  explicit HardwareClockMapper(double reset_threshold_ms = 5.0)
  : reset_threshold_ms_(reset_threshold_ms)
  {}

  HardwareClockMapping map(
    double device_ms, int64_t receive_ros_ns, int64_t receive_steady_ns)
  {
    if (!std::isfinite(device_ms) || receive_ros_ns <= 0 || receive_steady_ns <= 0) {
      return {0, false, false, "invalid_input", epoch_};
    }

    if (!initialized_) {
      return rebuild(device_ms, receive_ros_ns, receive_steady_ns, "initial_anchor");
    }

    const double device_delta_ms = device_ms - last_device_ms_;
    if (device_delta_ms < -reset_threshold_ms_) {
      return rebuild(device_ms, receive_ros_ns, receive_steady_ns, "device_clock_regression");
    }
    if (device_delta_ms < 0.0) {
      // A small regression is callback/frame reordering, not an epoch reset.
      // Its absolute capture time cannot be placed safely in this ordered ROS
      // stream, so expose it as invalid and leave the mapping state untouched.
      return {0, false, false, "device_frame_out_of_order", epoch_};
    }

    const double base_delta_ms = device_ms - base_device_ms_;
    const int64_t mapped_ns = base_ros_ns_ + static_cast<int64_t>(std::llround(base_delta_ms * 1e6));
    if (mapped_ns <= 0) {
      return {0, false, false, "mapped_time_out_of_range", epoch_};
    }

    // A small device-time reordering is preserved as such.  It is not a clock
    // epoch reset and must remain observable to callers.
    last_device_ms_ = device_ms;
    last_receive_steady_ns_ = receive_steady_ns;
    last_mapped_ns_ = mapped_ns;
    return {mapped_ns, true, false, "mapped", epoch_};
  }

  uint64_t rebuild_count() const {return rebuild_count_;}
  uint64_t epoch() const {return epoch_;}
  int64_t last_mapped_ns() const {return last_mapped_ns_;}

private:
  HardwareClockMapping rebuild(
    double device_ms, int64_t receive_ros_ns, int64_t receive_steady_ns,
    const char * reason)
  {
    // Re-anchoring to the independently observed host receive time preserves
    // the timestamp meaning after a device epoch reset.  If the system clock
    // itself moved behind the last mapped time, conversion is not reliable and
    // the frame is explicitly invalid instead of being forced monotonic.
    if (initialized_ && receive_ros_ns <= last_mapped_ns_) {
      return {0, false, false, "host_clock_not_monotonic_during_rebuild", epoch_};
    }
    base_device_ms_ = device_ms;
    base_ros_ns_ = receive_ros_ns;
    last_device_ms_ = device_ms;
    last_receive_steady_ns_ = receive_steady_ns;
    last_mapped_ns_ = receive_ros_ns;
    initialized_ = true;
    ++epoch_;
    if (epoch_ > 1) {
      ++rebuild_count_;
    }
    return {receive_ros_ns, true, true, reason, epoch_};
  }

  double reset_threshold_ms_;
  bool initialized_{false};
  double base_device_ms_{0.0};
  double last_device_ms_{0.0};
  int64_t base_ros_ns_{0};
  int64_t last_receive_steady_ns_{0};
  int64_t last_mapped_ns_{0};
  uint64_t epoch_{0};
  uint64_t rebuild_count_{0};
};

}  // namespace realsense2_camera

#endif  // REALSENSE2_CAMERA_HARDWARE_CLOCK_MAPPER_H
