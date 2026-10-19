#include "eddy_driver/eddy_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>

namespace eddy_driver
{

EddyNode::EddyNode(const rclcpp::NodeOptions & options)
: Node("eddy_driver_node", options)
{
  declare_parameter<std::string>("device", device_);
  declare_parameter<int>("baud", baud_);
  declare_parameter<std::string>("frame_id", frame_id_);
  declare_parameter<int>("queue_capacity", static_cast<int>(queue_capacity_));
  declare_parameter<double>("status_publish_rate", status_publish_rate_);
  declare_parameter<double>("settings_query_rate", settings_query_rate_);
  declare_parameter<std::string>("data_topic", data_topic_);
  declare_parameter<std::string>("channel_mean_topic", channel_mean_topic_);
  declare_parameter<std::string>("status_topic", status_topic_);
  declare_parameter<std::string>("command_topic", command_topic_);

  device_ = get_parameter("device").as_string();
  baud_ = get_parameter("baud").as_int();
  frame_id_ = get_parameter("frame_id").as_string();

  const int queue_capacity_param = get_parameter("queue_capacity").as_int();
  queue_capacity_ = static_cast<std::size_t>(std::max(1, queue_capacity_param));

  status_publish_rate_ = get_parameter("status_publish_rate").as_double();
  settings_query_rate_ = get_parameter("settings_query_rate").as_double();
  data_topic_ = get_parameter("data_topic").as_string();
  channel_mean_topic_ = get_parameter("channel_mean_topic").as_string();
  status_topic_ = get_parameter("status_topic").as_string();
  command_topic_ = get_parameter("command_topic").as_string();

  if (!std::isfinite(status_publish_rate_) || status_publish_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "status_publish_rate must be > 0, using 1.0 Hz");
    status_publish_rate_ = 1.0;
  }

  if (!std::isfinite(settings_query_rate_) || settings_query_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "settings_query_rate must be > 0, using 1.0 Hz");
    settings_query_rate_ = 1.0;
  }

  // 高速传感器数据流：尽力而为的 SensorDataQoS。
  data_pub_ = create_publisher<eddy_driver::msg::EddyFrame>(
    data_topic_, rclcpp::SensorDataQoS());
  channel_mean_pub_ = create_publisher<eddy_driver::msg::EddyChannelMean>(
    channel_mean_topic_, rclcpp::SensorDataQoS());

  // 低速设备状态：可靠队列。
  status_pub_ = create_publisher<eddy_driver::msg::EddyStatus>(status_topic_, 10);

  command_sub_ = create_subscription<eddy_driver::msg::EddyCommand>(
    command_topic_, 10,
    std::bind(&EddyNode::command_callback, this, std::placeholders::_1));

  if (!driver_.connect(device_, baud_, queue_capacity_)) {
    RCLCPP_FATAL(
      get_logger(), "Failed to open Eddy device %s at %d baud",
      device_.c_str(), baud_);
    throw std::runtime_error("Failed to connect Eddy device");
  }

  // 启动后恢复当前设备设置。
  driver_.send_command(EddyDriver::CMD_QUERY_AMPLITUDE, 0);
  driver_.send_command(EddyDriver::CMD_QUERY_FREQUENCY, 0);

  const auto status_period = std::chrono::duration<double>(1.0 / status_publish_rate_);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
    std::bind(&EddyNode::publish_status, this));

  const auto query_period = std::chrono::duration<double>(1.0 / settings_query_rate_);
  query_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(query_period),
    std::bind(&EddyNode::query_settings, this));

  data_thread_running_.store(true);
  data_thread_ = std::thread(&EddyNode::data_publish_loop, this);

  RCLCPP_INFO(
    get_logger(),
    "Eddy driver ready: %s @ %d, frame_id=%s, data=%s, channel_mean=%s, status=%s, cmd=%s",
    device_.c_str(), baud_, frame_id_.c_str(),
    data_topic_.c_str(), channel_mean_topic_.c_str(), status_topic_.c_str(),
    command_topic_.c_str());
}

EddyNode::~EddyNode()
{
  data_thread_running_.store(false);
  driver_.disconnect();

  if (data_thread_.joinable()) {
    data_thread_.join();
  }
}

void EddyNode::data_publish_loop()
{
  while (data_thread_running_.load() && rclcpp::ok()) {
    EddyDriver::AdcFrame frame{};

    if (!driver_.wait_for_frame(frame, 100)) {
      continue;
    }

    eddy_driver::msg::EddyFrame msg;

    // 时间戳在解析完一帧完整 UART 数据后立即记录。
    msg.header.stamp.sec = static_cast<int32_t>(frame.host_receive_time_ns / 1000000000ULL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(frame.host_receive_time_ns % 1000000000ULL);
    msg.header.frame_id = frame_id_;

    msg.sequence = frame.sequence;
    msg.samples_per_channel = static_cast<uint16_t>(EddyDriver::kSamples);
    msg.channel_count = static_cast<uint16_t>(EddyDriver::kChannels);

    int64_t sums[EddyDriver::kChannels]{};
    std::size_t output_index = 0;

    for (std::size_t s = 0; s < EddyDriver::kSamples; ++s) {
      for (std::size_t c = 0; c < EddyDriver::kChannels; ++c) {
        const int16_t value = frame.channels[s][c];
        msg.data[output_index++] = value;
        sums[c] += value;
      }
    }

    for (std::size_t c = 0; c < EddyDriver::kChannels; ++c) {
      msg.channel_mean[c] =
        static_cast<double>(sums[c]) / static_cast<double>(EddyDriver::kSamples);
    }

    msg.excitation_amplitude = driver_.current_amplitude();
    msg.excitation_frequency = driver_.current_frequency();
    msg.dropped_frames = driver_.dropped_frames();

    eddy_driver::msg::EddyChannelMean mean_msg;
    mean_msg.header = msg.header;
    mean_msg.channel_mean = msg.channel_mean;
    channel_mean_pub_->publish(mean_msg);
    data_pub_->publish(msg);
  }
}

void EddyNode::publish_status()
{
  eddy_driver::msg::EddyStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;

  msg.connected = driver_.is_connected();
  msg.current_amplitude = driver_.current_amplitude();
  msg.current_frequency = driver_.current_frequency();

  EddyDriver::StatusFrame status{};
  msg.has_status = driver_.get_status_snapshot(status);
  if (msg.has_status) {
    msg.last_status_cmd = status.cmd;
    msg.last_status_value = status.value;
    msg.last_status_is_error = status.is_error;
    msg.error_command = status.failed_command;
    msg.error_stage = status.error_stage;
    msg.error_hal_status = status.hal_status;
    msg.error_detail = status.detail;
  }

  msg.received_frames = driver_.received_frames();
  msg.dropped_frames = driver_.dropped_frames();
  msg.queue_depth = static_cast<uint32_t>(driver_.queue_depth());

  status_pub_->publish(msg);
}

void EddyNode::query_settings()
{
  if (!driver_.is_connected()) {
    return;
  }

  const bool amp_ok = driver_.send_command(EddyDriver::CMD_QUERY_AMPLITUDE, 0);
  const bool freq_ok = driver_.send_command(EddyDriver::CMD_QUERY_FREQUENCY, 0);

  if (!amp_ok || !freq_ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to query current excitation amplitude/frequency");
  }
}

bool EddyNode::validate_command(uint8_t command, uint32_t value) const
{
  switch (command) {
    case EddyDriver::CMD_SET_AMPLITUDE:
    case EddyDriver::CMD_SET_FREQUENCY:
      // data 是完整 uint32；由 STM32 按协议截断/限幅并通过读回响应确认实际值。
      (void)value;
      return true;

    case EddyDriver::CMD_QUERY_AMPLITUDE:
    case EddyDriver::CMD_QUERY_FREQUENCY:
      return true;

    default:
      return false;
  }
}

void EddyNode::command_callback(const eddy_driver::msg::EddyCommand::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!validate_command(msg->command, msg->value)) {
    RCLCPP_WARN(
      get_logger(), "Invalid Eddy command: cmd=0x%02X value=%u",
      msg->command, msg->value);
    return;
  }

  const uint32_t value =
    (msg->command == EddyDriver::CMD_QUERY_AMPLITUDE ||
     msg->command == EddyDriver::CMD_QUERY_FREQUENCY) ? 0 : msg->value;

  if (!driver_.send_command(msg->command, value)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to send Eddy command: cmd=0x%02X value=%u",
      msg->command, value);
  }
}

}  // namespace eddy_driver
