#include <cstdio>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "eddy_driver/eddy_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int result = 0;
  try {
    rclcpp::spin(std::make_shared<eddy_driver::EddyNode>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[EDDY] Fatal error: %s\n", e.what());
    result = 1;
  }

  rclcpp::shutdown();
  return result;
}
