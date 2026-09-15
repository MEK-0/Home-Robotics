#pragma once
#include "home_robotics_manipulation/object_registry.hpp"
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
namespace home_robotics_manipulation {
struct GripperGeometry {
  double hand_to_tcp;
  double hand_to_pad;
  double pad_half_height;
  double maximum_open_width;
};
struct GraspParameters {
  double pre_grasp_distance = 0.10;
  double grasp_clearance = 0.008;  // pad centre above cube centre
  Eigen::Vector3d approach_direction = -Eigen::Vector3d::UnitZ();
  double finger_open_width = 0.0;  // zero: derive from model limits; metadata only
  double expected_cube_width = 0.0;  // zero: derive from registry
  std::vector<std::string> allowed_touch_links = {"panda1_left_finger", "panda1_right_finger"};
};
struct GraspCandidate {
  std::string object_id, robot_id;
  geometry_msgs::msg::Pose pre_grasp_pose, grasp_pose;
  Eigen::Vector3d approach_vector, closing_axis;
  double pre_grasp_distance, expected_gripper_width, expected_cube_width;
  std::vector<std::string> allowed_touch_links;
};
GripperGeometry pandaGripperGeometry(const moveit::core::RobotModelConstPtr&);
GraspCandidate generateTopDownGrasp(const ObjectMetadata&, const geometry_msgs::msg::Pose&,
                                   const GripperGeometry&, const GraspParameters& = {});
bool pregraspJointNamesAllowed(const std::vector<std::string>& names);
// Returns a private ACM copy; never writes a PlanningScene or SRDF.
collision_detection::AllowedCollisionMatrix graspEvaluationACM(
  const collision_detection::AllowedCollisionMatrix&, const GraspCandidate&);
}
