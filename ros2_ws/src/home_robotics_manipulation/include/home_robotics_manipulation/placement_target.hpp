#pragma once
#include "home_robotics_manipulation/manipulation_types.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
namespace home_robotics_manipulation {
// Read-only view of scene.yaml. Surface pose Z is its TOP, not box centre.
struct PlacementTarget {
  std::string target_id, support_surface_id;
  Eigen::Vector2d center, size;
  double surface_top, clearance=0.001, approach_clearance=0.10;
  Eigen::Vector3d approach_direction{0,0,-1};
  std::string orientation_policy="preserve";
  static PlacementTarget lookup(const std::string& path,const std::string& id) {
    auto s=YAML::LoadFile(path)["scene"]["surfaces"][id];
    if(!s || s["workspace"].as<std::string>()!="panda1_primary_workspace")
      throw std::runtime_error("PLACE_FAILED invalid Panda1 target: "+id);
    auto p=s["pose"]["position"].as<std::vector<double>>();
    auto q=s["pose"]["quaternion_wxyz"].as<std::vector<double>>();
    auto r=s["safe_place_region"];
    auto c=r["center"].as<std::vector<double>>(),size=r["size"].as<std::vector<double>>();
    const double top=s["top_height"].as<double>();
    if(q!=std::vector<double>({1,0,0,0}) || r["frame"].as<std::string>()!=id ||
       s["pose"]["frame"].as<std::string>()!="world" ||
       !std::isfinite(top) || std::abs(top-p.at(2))>1e-8 || size.at(0)<=0 || size.at(1)<=0)
      throw std::runtime_error("PLACE_FAILED unsupported surface geometry");
    return {id,id,{p.at(0)+c.at(0),p.at(1)+c.at(1)},{size[0],size[1]},top};
  }
  Eigen::Vector3d halfExtents(const ObjectMetadata& object,const geometry_msgs::msg::Quaternion& q) const {
    if(object.geometry.primitives.size()!=1 || object.collision_shape!=CollisionShapeType::BOX)
      throw std::runtime_error("PLACE_FAILED unsupported placement geometry");
    const auto& d=object.geometry.primitives[0].dimensions;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    if(!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>1e-6)
      throw std::runtime_error("INVALID_OBJECT_STATE orientation");
    return rotation.toRotationMatrix().cwiseAbs()*Eigen::Vector3d(d.at(0),d.at(1),d.at(2))/2;
  }
  geometry_msgs::msg::Pose objectPose(const ObjectMetadata& object,const geometry_msgs::msg::Quaternion& orientation) const {
    auto e=halfExtents(object,orientation);
    if((size.array()/2<=e.head<2>().array()).any())throw std::runtime_error("PLACE_FAILED object too large");
    geometry_msgs::msg::Pose p;p.orientation=orientation;p.position.x=center.x();p.position.y=center.y();
    p.position.z=surface_top+e.z()+clearance;return p;
  }
  bool contains(const ObjectMetadata& object,const geometry_msgs::msg::Pose& p) const {
    auto e=halfExtents(object,p.orientation);
    return std::abs(p.position.x-center.x())+e.x()<=size.x()/2 &&
      std::abs(p.position.y-center.y())+e.y()<=size.y()/2 &&
      std::abs(p.position.z-e.z()-surface_top)<=0.003;
  }
  geometry_msgs::msg::Pose prePlace(geometry_msgs::msg::Pose tcp) const {
    tcp.position.z+=approach_clearance;return tcp;
  }
};
}
