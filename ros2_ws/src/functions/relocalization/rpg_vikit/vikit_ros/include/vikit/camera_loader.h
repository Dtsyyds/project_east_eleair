/* Vikit ROS2 camera loading. Original authors: cforster, StrangeFly. */
#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_
#include <cmath>
#include <stdexcept>
#include <memory>
#include <vector>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>
namespace vk { namespace camera_loader {
template<class T> T value(const rclcpp::Node::SharedPtr &node, const std::string &ns,
                         const std::string &key, const std::string &alias, T fallback) {
  const auto &overrides=node->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto &name : {ns.empty()?key:ns+"."+key, ns.empty()?alias:ns+"."+alias, key, alias}) {
    if (node->has_parameter(name)) return node->get_parameter(name).template get_value<T>();
    const auto found=overrides.find(name);
    if (found!=overrides.end()) return node->declare_parameter<T>(name,fallback);
  }
  return fallback;
}
inline bool loadFromRosNs(const rclcpp::Node::SharedPtr &node, const std::string &ns, vk::AbstractCamera *&cam) {
  const auto model=value<std::string>(node,ns,"model","cam_model","");
  if (model=="Ocam") {
    const auto file=value<std::string>(node,ns,"calib_file","cam_calib_file","");
    if (file.empty()) throw std::invalid_argument("Ocam requires calib_file.");
    cam=new vk::OmniCamera(file); return true;
  }
  const int width=value<int>(node,ns,"width","cam_width",0), height=value<int>(node,ns,"height","cam_height",0);
  const double scale=value<double>(node,ns,"scale","cam_scale",1.0);
  const double fx=value<double>(node,ns,"fx","cam_fx",0), fy=value<double>(node,ns,"fy","cam_fy",0);
  const double cx=value<double>(node,ns,"cx","cam_cx",0), cy=value<double>(node,ns,"cy","cam_cy",0);
  if (width<=0 || height<=0 || !std::isfinite(scale) || scale<=0 || !std::isfinite(fx) || fx<=0 ||
      !std::isfinite(fy) || fy<=0 || !std::isfinite(cx) || !std::isfinite(cy))
    throw std::invalid_argument("Camera requires positive resolution, scale and focal lengths and finite principal point.");
  auto d=[&](const std::string &key) { double v=value<double>(node,ns,key,"cam_"+key,0.0); if(!std::isfinite(v)) throw std::invalid_argument("Camera coefficients must be finite."); return v; };
  if (model=="Pinhole") cam=new vk::PinholeCamera(width,height,scale,fx,fy,cx,cy,d("d0"),d("d1"),d("d2"),d("d3"));
  else if (model=="EquidistantCamera") cam=new vk::EquidistantCamera(width,height,scale,fx,fy,cx,cy,d("k1"),d("k2"),d("k3"),d("k4"));
  else if (model=="PolynomialCamera" || model=="FishPoly") cam=new vk::PolynomialCamera(width,height,fx,fy,cx,cy,d("skew"),d("k2"),d("k3"),d("k4"),d("k5"),d("k6"),d("k7"));
  else if (model=="ATAN") cam=new vk::ATANCamera(width,height,fx,fy,cx,cy,d("d0"));
  else { cam=nullptr; return false; }
  return true;
}
inline bool loadFromRosNs(const rclcpp::Node::SharedPtr &node, const std::string &ns, std::vector<vk::AbstractCamera*> &cameras) {
  const int count=value<int>(node,ns,"num","cam_num",0);
  if(count<1) return false;
  std::vector<std::unique_ptr<vk::AbstractCamera>> loaded;
  for(int i=0;i<count;++i) { vk::AbstractCamera *camera=nullptr;
    if(!loadFromRosNs(node,ns+".cam_"+std::to_string(i),camera)) return false;
    loaded.emplace_back(camera);
  }
  for(auto &camera:loaded) cameras.push_back(camera.release());
  return true;
}
}} // namespace vk::camera_loader
#endif
