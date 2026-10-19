#ifndef EDDY_DRIVER__EDDY_NODE_HPP_
#define EDDY_DRIVER__EDDY_NODE_HPP_

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "eddy_driver/eddy_driver.hpp"
#include "eddy_driver/msg/eddy_channel_mean.hpp"
#include "eddy_driver/msg/eddy_command.hpp"
#include "eddy_driver/msg/eddy_frame.hpp"
#include "eddy_driver/msg/eddy_status.hpp"

namespace eddy_driver
{

class EddyNode : public rclcpp::Node
{
public:
  explicit EddyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EddyNode() override;

private:
  void data_publish_loop();
  void publish_status();
  void query_settings();

  void command_callback(const eddy_driver::msg::EddyCommand::SharedPtr msg);
  bool validate_command(uint8_t command, uint32_t value) const;

  EddyDriver driver_;

  std::string device_{"/dev/eddy"};
  int baud_{4000000};
  std::string frame_id_{"eddy_probe_link"};
  std::size_t queue_capacity_{256};

  double status_publish_rate_{1.0};
  double settings_query_rate_{1.0};

  std::string data_topic_{"/eddy/data"};
  std::string channel_mean_topic_{"/eddy/channel_mean"};
  std::string status_topic_{"/eddy/status"};
  std::string command_topic_{"/eddy/cmd"};

  std::atomic<bool> data_thread_running_{false};
  std::thread data_thread_;

  rclcpp::Publisher<eddy_driver::msg::EddyFrame>::SharedPtr data_pub_;
  rclcpp::Publisher<eddy_driver::msg::EddyChannelMean>::SharedPtr channel_mean_pub_;
  rclcpp::Publisher<eddy_driver::msg::EddyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<eddy_driver::msg::EddyCommand>::SharedPtr command_sub_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr query_timer_;
};

}  // namespace eddy_driver

#endif  // EDDY_DRIVER__EDDY_NODE_HPP_
