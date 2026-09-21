#pragma once
#include "home_robotics_manipulation/grasp_pose_generator.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <thread>
#include <set>
using namespace std::chrono_literals;
namespace hm = home_robotics_manipulation;
namespace {
void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& p) {
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  t.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).toRotationMatrix(); return t;
}
void print_pose(const rclcpp::Logger& log, const char* label, const geometry_msgs::msg::Pose& p) {
  RCLCPP_INFO(log,"%s xyz=[%.12f, %.12f, %.12f] quaternion_xyzw=[%.12f, %.12f, %.12f, %.12f]", label,
    p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
}
double separation(const moveit::core::RobotState& s) {
  return s.getVariablePosition("panda2_rail_joint")-s.getVariablePosition("panda1_rail_joint");
}
bool valid(planning_scene::PlanningScene& scene, moveit::core::RobotState& s,
           const collision_detection::AllowedCollisionMatrix& acm, const rclcpp::Logger& log,
           const std::string& label, bool verbose) {
  s.update(); collision_detection::CollisionRequest request; collision_detection::CollisionResult result;
  request.contacts=true; request.max_contacts=1000; request.max_contacts_per_pair=1;
  scene.checkCollision(request,result,s,acm);
  const bool bounds=s.satisfiesBounds(1e-6), rail=separation(s)>=0.7;
  if (verbose) {
    RCLCPP_INFO(log,"%s validity=%s bounds=%s separation=%.9f contacts=%zu",label.c_str(),
      (!result.collision && bounds && rail)?"VALID":"INVALID",bounds?"OK":"FAIL",separation(s),result.contacts.size());
    for (const auto& [pair, contacts]:result.contacts) RCLCPP_INFO(log,"%s collision: %s <-> %s",label.c_str(),pair.first.c_str(),pair.second.c_str());
  }
  return !result.collision && bounds && rail;
}
void fresh_state(moveit::core::RobotState& s,const rclcpp::Node::SharedPtr& reader,
                 const rclcpp::Node::SharedPtr& clock_node) {
  const auto deadline=std::chrono::steady_clock::now()+30s;
  std::string reason="no /joint_states message";
  while(rclcpp::ok() && std::chrono::steady_clock::now()<deadline) {
    sensor_msgs::msg::JointState msg;
    if(!rclcpp::wait_for_message(msg,reader,"/joint_states",1s)) continue;
    if(msg.name.size()!=msg.position.size()){reason="malformed joint arrays";continue;}
    const auto now=clock_node->now();
    const rclcpp::Time stamp(msg.header.stamp,now.get_clock_type());
    const double age=(now-stamp).seconds();
    if(now.nanoseconds()==0 || stamp.nanoseconds()==0 || age < -0.1 || age > 0.5){
      reason="clock not ready or stale /joint_states; age="+std::to_string(age);continue;
    }
    std::map<std::string,double> values;
    for(std::size_t i=0;i<msg.name.size();++i) values[msg.name[i]]=msg.position[i];
    bool complete=true;
    // Require both robots and fingers as well as all eight Panda1 planning joints.
    for(const auto& name:s.getVariableNames()) {
      if(!values.count(name) || !std::isfinite(values.at(name))){
        reason="missing/nonfinite joint "+name;complete=false;break;
      }
    }
    if(!complete)continue;
    s.setVariablePositions(values);s.update();
    RCLCPP_INFO(clock_node->get_logger(),"Fresh authoritative /joint_states: stamp=%.9f age=%.6f s; all robot variables present",stamp.seconds(),age);
    return;
  }
  throw std::runtime_error("ROBOT_STATE_TIMEOUT after 30 s: "+reason);
}
void print_joints(const rclcpp::Logger& log,const char* label,const moveit::core::RobotState& s) {
  for (const auto& name:s.getJointModelGroup("panda1_manipulator")->getVariableNames())
    RCLCPP_INFO(log,"%s %s=%.12f",label,name.c_str(),s.getVariablePosition(name));
}
visualization_msgs::msg::MarkerArray markers(const hm::GraspCandidate& c) {
  visualization_msgs::msg::MarkerArray array;
  for (int i=0;i<2;++i) {
    visualization_msgs::msg::Marker m; m.header.frame_id="world"; m.ns="grasp_poses"; m.id=i;
    m.type=m.SPHERE; m.action=m.ADD; m.pose=i==0?c.pre_grasp_pose:c.grasp_pose;
    m.scale.x=m.scale.y=m.scale.z=0.015; m.color.a=1; m.color.g=i==0?1:0; m.color.r=i==1?1:0;
    m.color.b=i==1?1:0; array.markers.push_back(m);
  }
  visualization_msgs::msg::Marker a; a.header.frame_id="world"; a.ns="approach"; a.id=0;
  a.type=a.ARROW; a.action=a.ADD; a.pose.orientation.w=1;
  a.points={c.pre_grasp_pose.position,c.grasp_pose.position}; a.scale.x=0.005; a.scale.y=0.012; a.scale.z=0.018;
  a.color.a=1; a.color.b=1; a.color.g=0.7; array.markers.push_back(a);
  auto closing=a; closing.ns="closing_axis"; closing.id=0; closing.color.r=1; closing.color.g=0.7; closing.color.b=0;
  auto left=c.grasp_pose.position, right=left;
  left.x-=(c.expected_gripper_width/2)*c.closing_axis.x();left.y-=(c.expected_gripper_width/2)*c.closing_axis.y();
  right.x+=(c.expected_gripper_width/2)*c.closing_axis.x();right.y+=(c.expected_gripper_width/2)*c.closing_axis.y();
  closing.points={left,right};array.markers.push_back(closing);return array;
}
}
