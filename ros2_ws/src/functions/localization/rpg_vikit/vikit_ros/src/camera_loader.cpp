/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 *  Update on: Feb 01, 2025
 *      Author: StrangeFly
 */

#include <vikit/camera_loader.h>

namespace vk {
namespace camera_loader {

template<typename T>
T getRemoteParamWithAlias(const rclcpp::Node::SharedPtr & nh, const std::string & ns,
                          const std::string & name, const std::string & alias,
                          const T & default_value)
{
  T value;
  if (nh->get_parameter(ns + "." + name, value) || nh->get_parameter(name, value)) return value;
  if (nh->get_parameter(ns + "." + alias, value) || nh->get_parameter(alias, value)) return value;
  return default_value;
}

/// Load from ROS Namespace
bool loadFromRosNs(const rclcpp::Node::SharedPtr & nh, const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  std::string cam_model(getRemoteParamWithAlias<std::string>(nh, ns, "cam_model", "model", ""));
  if(cam_model == "Ocam")
  {
    cam = new vk::OmniCamera(getRemoteParam<std::string>(nh, ns, "cam_calib_file", ""));
  }
  else if(cam_model == "Pinhole")
  {
    cam = new vk::PinholeCamera(
        getRemoteParamWithAlias<int>(nh, ns, "cam_width", "width", 0),
        getRemoteParamWithAlias<int>(nh, ns, "cam_height", "height", 0),
        getRemoteParamWithAlias<double>(nh, ns, "scale", "scale", 1.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_fx", "fx", 0.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_fy", "fy", 0.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_cx", "cx", 0.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_cy", "cy", 0.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_d0", "d0", 0.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_d1", "d1", 0.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_d2", "d2", 0.0),
        getRemoteParamWithAlias<double>(nh, ns, "cam_d3", "d3", 0.0));
  }
  else if(cam_model == "EquidistantCamera")
  {
    cam = new vk::EquidistantCamera(
        getParam<int>(nh, ns+"/cam_width"),
        getParam<int>(nh, ns+"/cam_height"),
        getParam<double>(nh, ns+"/scale", 1.0),
        getParam<double>(nh, ns+"/cam_fx"),
        getParam<double>(nh, ns+"/cam_fy"),
        getParam<double>(nh, ns+"/cam_cx"),
        getParam<double>(nh, ns+"/cam_cy"),
        getParam<double>(nh, ns+"/k1", 0.0),
        getParam<double>(nh, ns+"/k2", 0.0),
        getParam<double>(nh, ns+"/k3", 0.0),
        getParam<double>(nh, ns+"/k4", 0.0));
  }
  else if(cam_model == "PolynomialCamera")
  {
    cam = new vk::PolynomialCamera(
        getParam<int>(nh, ns+"/cam_width"),
        getParam<int>(nh, ns+"/cam_height"),
        // getParam<double>(nh, ns+"/scale", 1.0),
        getParam<double>(nh, ns+"/cam_fx"),
        getParam<double>(nh, ns+"/cam_fy"),
        getParam<double>(nh, ns+"/cam_cx"),
        getParam<double>(nh, ns+"/cam_cy"),
        getParam<double>(nh, ns+"/cam_skew"),
        getParam<double>(nh, ns+"/k2", 0.0),
        getParam<double>(nh, ns+"/k3", 0.0),
        getParam<double>(nh, ns+"/k4", 0.0),
        getParam<double>(nh, ns+"/k5", 0.0),
        getParam<double>(nh, ns+"/k6", 0.0),
        getParam<double>(nh, ns+"/k7", 0.0));
  }
  else if(cam_model == "ATAN")
  {
    cam = new vk::ATANCamera(
        getParam<int>(nh, ns+"/cam_width"),
        getParam<int>(nh, ns+"/cam_height"),
        getParam<double>(nh, ns+"/cam_fx"),
        getParam<double>(nh, ns+"/cam_fy"),
        getParam<double>(nh, ns+"/cam_cx"),
        getParam<double>(nh, ns+"/cam_cy"),
        getParam<double>(nh, ns+"/cam_d0"));
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

bool loadFromRosNs(const rclcpp::Node::SharedPtr & nh, const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  bool res = true;
  std::string cam_model(getParam<std::string>(nh, ns+"/cam_model", "Pinhole"));
  int cam_num = getParam<int>(nh, ns+"/cam_num");
  for (int i = 0; i < cam_num; i ++)
  {
    std::string cam_ns = ns + "/cam_" + std::to_string(i);
    std::string cam_model(getParam<std::string>(nh, cam_ns+"/cam_model"));
    if(cam_model == "FishPoly")
    {
      cam_list.push_back(new vk::PolynomialCamera(
        getParam<int>(nh, cam_ns+"/image_width"),
        getParam<int>(nh, cam_ns+"/image_height"),
        // getParam<double>(nh, cam_ns+"/scale", 1.0),
        getParam<double>(nh, cam_ns+"/A11"),  // cam_fx
        getParam<double>(nh, cam_ns+"/A22"),  // cam_fy
        getParam<double>(nh, cam_ns+"/u0"),  // cam_cx
        getParam<double>(nh, cam_ns+"/v0"),  // cam_cy
        getParam<double>(nh, cam_ns+"/A12"), // cam_skew
        getParam<double>(nh, cam_ns+"/k2", 0.0),
        getParam<double>(nh, cam_ns+"/k3", 0.0),
        getParam<double>(nh, cam_ns+"/k4", 0.0),
        getParam<double>(nh, cam_ns+"/k5", 0.0),
        getParam<double>(nh, cam_ns+"/k6", 0.0),
        getParam<double>(nh, cam_ns+"/k7", 0.0)));
    }
    else if(cam_model == "Pinhole")
    {
      cam_list.push_back(new vk::PinholeCamera(
          getParam<int>(nh, ns+"/cam_width"),
          getParam<int>(nh, ns+"/cam_height"),
          getParam<double>(nh, ns+"/scale", 1.0),
          getParam<double>(nh, ns+"/cam_fx"),
          getParam<double>(nh, ns+"/cam_fy"),
          getParam<double>(nh, ns+"/cam_cx"),
          getParam<double>(nh, ns+"/cam_cy"),
          getParam<double>(nh, ns+"/cam_d0", 0.0),
          getParam<double>(nh, ns+"/cam_d1", 0.0),
          getParam<double>(nh, ns+"/cam_d2", 0.0),
          getParam<double>(nh, ns+"/cam_d3", 0.0)));
    }
    else 
    {
      // cam_list.clear();
      res = false;
    }
  }
  
  return res;
}

} // namespace camera_loader
} // namespace vk
