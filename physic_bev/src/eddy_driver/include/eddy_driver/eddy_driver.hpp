#ifndef EDDY_DRIVER__EDDY_DRIVER_HPP_
#define EDDY_DRIVER__EDDY_DRIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace eddy_driver
{

class EddyDriver
{
public:
  static constexpr std::size_t kSamples = 20;
  static constexpr std::size_t kChannels = 8;
  static constexpr std::size_t kAdcPayloadSize = kSamples * kChannels * sizeof(int16_t);
  static constexpr std::size_t kAdcFrameSize = 2 + kAdcPayloadSize + 2;  // 324 字节
  static constexpr std::size_t kCmdFrameSize = 7;

  static constexpr uint8_t CMD_SET_AMPLITUDE = 0x01;
  static constexpr uint8_t CMD_SET_FREQUENCY = 0x02;
  static constexpr uint8_t CMD_QUERY_AMPLITUDE = 0x11;
  static constexpr uint8_t CMD_QUERY_FREQUENCY = 0x12;
  static constexpr uint8_t RESP_SET_AMPLITUDE = 0x81;
  static constexpr uint8_t RESP_SET_FREQUENCY = 0x82;
  static constexpr uint8_t RESP_ERROR = 0xE0;

  // STM32 固件会将设置值限制在这些实际生效范围内。
  static constexpr uint32_t kMinAmplitude = 1;
  static constexpr uint32_t kMaxAmplitude = 800;
  static constexpr uint32_t kMinFrequency = 5;
  static constexpr uint32_t kMaxFrequency = 216000000;

  struct AdcFrame
  {
    uint32_t sequence{0};
    uint64_t host_receive_time_ns{0};
    int16_t channels[kSamples][kChannels]{};
  };

  struct StatusFrame
  {
    uint8_t cmd{0};
    uint32_t value{0};
    bool is_error{false};
    uint8_t failed_command{0};
    uint8_t error_stage{0};
    uint8_t hal_status{0};
    uint8_t detail{0};
  };

  EddyDriver();
  ~EddyDriver();

  EddyDriver(const EddyDriver &) = delete;
  EddyDriver & operator=(const EddyDriver &) = delete;

  bool connect(
    const std::string & device,
    int baud = 4000000,
    std::size_t queue_capacity = 256);

  void disconnect();
  bool is_connected() const;

  // 带超时的阻塞等待。帧按 FIFO 顺序逐个取出。
  bool wait_for_frame(AdcFrame & frame, int timeout_ms);

  // 线程安全的 STM32 命令发送。
  bool send_command(uint8_t cmd, uint32_t value = 0);

  uint32_t current_amplitude() const;
  uint32_t current_frequency() const;

  bool get_status_snapshot(StatusFrame & status) const;

  uint64_t received_frames() const;
  uint64_t dropped_frames() const;
  std::size_t queue_depth() const;

private:
  bool open_serial(const std::string & device, int baud);
  void close_serial();

  void receive_loop();
  void parse_stream(std::vector<uint8_t> & raw_buf);
  bool parse_adc_frame(const uint8_t * data, std::size_t size);
  void parse_status_frame(const uint8_t * data);
  void push_frame(const AdcFrame & frame);
  void set_status(const StatusFrame & status);

  int fd_{-1};
  std::atomic<bool> running_{false};
  std::thread receive_thread_;

  mutable std::mutex tx_mutex_;

  mutable std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::deque<AdcFrame> frame_queue_;
  std::size_t queue_capacity_{256};

  mutable std::mutex status_mutex_;
  StatusFrame latest_status_{};
  bool has_status_{false};

  std::atomic<uint32_t> current_amplitude_{0};
  std::atomic<uint32_t> current_frequency_{0};

  std::atomic<uint32_t> sequence_{0};
  std::atomic<uint64_t> received_frames_{0};
  std::atomic<uint64_t> dropped_frames_{0};
};

}  // namespace eddy_driver

#endif  // EDDY_DRIVER__EDDY_DRIVER_HPP_
