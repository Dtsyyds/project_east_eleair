#include "eddy_driver/eddy_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace eddy_driver
{
namespace
{
constexpr uint8_t ADC_HEAD0 = 0x55;
constexpr uint8_t ADC_HEAD1 = 0xAA;
constexpr uint8_t ADC_TAIL0 = 0x7E;
constexpr uint8_t ADC_TAIL1 = 0xFE;
constexpr uint8_t CMD_HEAD = 0xAA;
constexpr uint8_t CMD_TAIL = 0x55;

uint64_t system_time_now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}
}  // namespace

EddyDriver::EddyDriver() = default;

EddyDriver::~EddyDriver()
{
  disconnect();
}

bool EddyDriver::connect(
  const std::string & device,
  int baud,
  std::size_t queue_capacity)
{
  if (is_connected()) {
    return true;
  }

  queue_capacity_ = std::max<std::size_t>(1, queue_capacity);

  if (!open_serial(device, baud)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_queue_.clear();
  }

  sequence_.store(0);
  received_frames_.store(0);
  dropped_frames_.store(0);

  running_.store(true);
  receive_thread_ = std::thread(&EddyDriver::receive_loop, this);

  return true;
}

void EddyDriver::disconnect()
{
  running_.store(false);
  frame_cv_.notify_all();

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  close_serial();
}

bool EddyDriver::is_connected() const
{
  return fd_ >= 0;
}

bool EddyDriver::open_serial(const std::string & device, int baud)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) {
    std::fprintf(
      stderr, "[EDDY] Failed to open %s: %s\n",
      device.c_str(), std::strerror(errno));
    return false;
  }

  const int flags = fcntl(fd_, F_GETFL);
  if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    std::fprintf(stderr, "[EDDY] Failed to set non-blocking mode: %s\n", std::strerror(errno));
    close_serial();
    return false;
  }

  struct termios options {};
  if (tcgetattr(fd_, &options) != 0) {
    std::fprintf(stderr, "[EDDY] tcgetattr failed: %s\n", std::strerror(errno));
    close_serial();
    return false;
  }

  speed_t baud_const = B115200;
  if (baud == 115200) {
    baud_const = B115200;
  } else if (baud == 4000000) {
#ifdef B4000000
    baud_const = B4000000;
#else
    std::fprintf(stderr, "[EDDY] This platform does not define B4000000.\n");
    close_serial();
    return false;
#endif
  } else {
    std::fprintf(stderr, "[EDDY] Unsupported baud rate: %d\n", baud);
    close_serial();
    return false;
  }

  cfsetispeed(&options, baud_const);
  cfsetospeed(&options, baud_const);

  // 8N1，无流控，原始模式。
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag |= (CREAD | CLOCAL);

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNBRK);
  options.c_oflag &= ~OPOST;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    std::fprintf(stderr, "[EDDY] tcsetattr failed: %s\n", std::strerror(errno));
    close_serial();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  std::printf("[EDDY] Opened %s @ %d baud\n", device.c_str(), baud);
  return true;
}

void EddyDriver::close_serial()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
    std::printf("[EDDY] Serial closed\n");
  }
}

void EddyDriver::receive_loop()
{
  std::vector<uint8_t> raw_buf;
  raw_buf.reserve(8192);

  uint8_t buffer[4096];

  while (running_.load()) {
    const ssize_t bytes_read = ::read(fd_, buffer, sizeof(buffer));

    if (bytes_read > 0) {
      raw_buf.insert(raw_buf.end(), buffer, buffer + bytes_read);
      parse_stream(raw_buf);
    } else {
      if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        std::fprintf(stderr, "[EDDY] Serial read error: %s\n", std::strerror(errno));
      }
      usleep(100);
    }
  }
}

void EddyDriver::parse_stream(std::vector<uint8_t> & raw_buf)
{
  while (!raw_buf.empty()) {
    // AD7606 数据帧：55 AA | 20 x 8 x int16 小端序 | 7E FE。
    if (raw_buf[0] == ADC_HEAD0) {
      if (raw_buf.size() < 2) {
        return;
      }
      if (raw_buf[1] != ADC_HEAD1) {
        raw_buf.erase(raw_buf.begin());
        continue;
      }
      if (raw_buf.size() < kAdcFrameSize) {
        return;
      }
      if (
        raw_buf[kAdcFrameSize - 2] != ADC_TAIL0 ||
        raw_buf[kAdcFrameSize - 1] != ADC_TAIL1)
      {
        raw_buf.erase(raw_buf.begin());
        continue;
      }
      parse_adc_frame(raw_buf.data(), kAdcFrameSize);
      raw_buf.erase(raw_buf.begin(), raw_buf.begin() + kAdcFrameSize);
      continue;
    }

    // 控制响应：AA | CMD | uint32 小端序数据 | 55。
    if (raw_buf[0] == CMD_HEAD) {
      if (raw_buf.size() < kCmdFrameSize) {
        return;
      }
      if (raw_buf[kCmdFrameSize - 1] != CMD_TAIL) {
        raw_buf.erase(raw_buf.begin());
        continue;
      }
      parse_status_frame(raw_buf.data());
      raw_buf.erase(raw_buf.begin(), raw_buf.begin() + kCmdFrameSize);
      continue;
    }

    // 未知字节：丢弃，直到找到已知帧头。
    raw_buf.erase(raw_buf.begin());
  }
}

bool EddyDriver::parse_adc_frame(const uint8_t * data, std::size_t size)
{
  if (
    size != kAdcFrameSize || data[0] != ADC_HEAD0 || data[1] != ADC_HEAD1 ||
    data[kAdcFrameSize - 2] != ADC_TAIL0 || data[kAdcFrameSize - 1] != ADC_TAIL1)
  {
    return false;
  }

  AdcFrame frame{};
  frame.sequence = sequence_.fetch_add(1);
  frame.host_receive_time_ns = system_time_now_ns();

  std::size_t byte_index = 2;
  for (std::size_t sample = 0; sample < kSamples; ++sample) {
    for (std::size_t channel = 0; channel < kChannels; ++channel) {
      const uint8_t low = data[byte_index++];
      const uint8_t high = data[byte_index++];
      frame.channels[sample][channel] = static_cast<int16_t>(
        static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8));
    }
  }

  received_frames_.fetch_add(1);
  push_frame(frame);
  return true;
}

void EddyDriver::parse_status_frame(const uint8_t * data)
{
  StatusFrame status{};
  status.cmd = data[1];
  status.value =
    static_cast<uint32_t>(data[2]) |
    (static_cast<uint32_t>(data[3]) << 8) |
    (static_cast<uint32_t>(data[4]) << 16) |
    (static_cast<uint32_t>(data[5]) << 24);

  if (status.cmd == RESP_ERROR) {
    status.is_error = true;
    status.failed_command = data[2];
    status.error_stage = data[3];
    status.hal_status = data[4];
    status.detail = data[5];
  }

  set_status(status);
}

void EddyDriver::push_frame(const AdcFrame & frame)
{
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);

    if (frame_queue_.size() >= queue_capacity_) {
      frame_queue_.pop_front();
      dropped_frames_.fetch_add(1);
    }

    frame_queue_.push_back(frame);
  }

  frame_cv_.notify_one();
}

bool EddyDriver::wait_for_frame(AdcFrame & frame, int timeout_ms)
{
  std::unique_lock<std::mutex> lock(frame_mutex_);

  const bool ready = frame_cv_.wait_for(
    lock,
    std::chrono::milliseconds(std::max(0, timeout_ms)),
    [this]() {
      return !frame_queue_.empty() || !running_.load();
    });

  if (!ready || frame_queue_.empty()) {
    return false;
  }

  frame = frame_queue_.front();
  frame_queue_.pop_front();
  return true;
}

bool EddyDriver::send_command(uint8_t cmd, uint32_t value)
{
  if (!is_connected()) {
    return false;
  }

  uint8_t packet[kCmdFrameSize]{};
  packet[0] = CMD_HEAD;
  packet[1] = cmd;
  packet[2] = static_cast<uint8_t>(value & 0xFF);
  packet[3] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  packet[4] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  packet[5] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
  packet[6] = CMD_TAIL;

  std::lock_guard<std::mutex> lock(tx_mutex_);
  std::size_t bytes_written = 0;
  while (bytes_written < sizeof(packet)) {
    const ssize_t n = ::write(fd_, packet + bytes_written, sizeof(packet) - bytes_written);
    if (n > 0) {
      bytes_written += static_cast<std::size_t>(n);
      continue;
    }

    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd writable{};
      writable.fd = fd_;
      writable.events = POLLOUT;
      if (::poll(&writable, 1, 1000) > 0) {
        continue;
      }
    }

    std::fprintf(stderr, "[EDDY] Command write failed: %s\n", std::strerror(errno));
    return false;
  }

  return true;
}

void EddyDriver::set_status(const StatusFrame & status)
{
  if (!status.is_error) {
    if (status.cmd == RESP_SET_AMPLITUDE || status.cmd == CMD_QUERY_AMPLITUDE) {
      current_amplitude_.store(status.value);
    } else if (status.cmd == RESP_SET_FREQUENCY || status.cmd == CMD_QUERY_FREQUENCY) {
      current_frequency_.store(status.value);
    }
  }

  std::lock_guard<std::mutex> lock(status_mutex_);
  latest_status_ = status;
  has_status_ = true;
}

uint32_t EddyDriver::current_amplitude() const
{
  return current_amplitude_.load();
}

uint32_t EddyDriver::current_frequency() const
{
  return current_frequency_.load();
}

bool EddyDriver::get_status_snapshot(StatusFrame & status) const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (!has_status_) {
    return false;
  }
  status = latest_status_;
  return true;
}

uint64_t EddyDriver::received_frames() const
{
  return received_frames_.load();
}

uint64_t EddyDriver::dropped_frames() const
{
  return dropped_frames_.load();
}

std::size_t EddyDriver::queue_depth() const
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  return frame_queue_.size();
}

}  // namespace eddy_driver
