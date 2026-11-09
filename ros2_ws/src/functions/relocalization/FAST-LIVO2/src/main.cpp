#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  try
  {
    rclcpp::Node::SharedPtr nh;
    LIVMapper mapper(nh, "laserMapping", options);
    try
    {
      mapper.initializeSubscribersAndPublishers(nh);
      mapper.run(nh);
    }
    catch (...)
    {
      // Stop ROS before unwinding mapper's TF listener on initialization/runtime failure.
      rclcpp::shutdown();
      throw;
    }
    rclcpp::shutdown();
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("laserMapping"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
