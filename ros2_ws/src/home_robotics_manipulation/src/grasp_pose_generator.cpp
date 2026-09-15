#include "home_robotics_manipulation/grasp_pose_generator.hpp"
#include <moveit/robot_state/robot_state.hpp>
#include <geometric_shapes/shapes.h>
#include <cmath>
#include <set>
#include <stdexcept>
namespace home_robotics_manipulation {
namespace {
void require(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
geometry_msgs::msg::Pose pose(const Eigen::Vector3d& p, const Eigen::Matrix3d& r) {
  geometry_msgs::msg::Pose result;
  result.position.x=p.x(); result.position.y=p.y(); result.position.z=p.z();
  Eigen::Quaterniond q(r); q.normalize();
  result.orientation.x=q.x(); result.orientation.y=q.y(); result.orientation.z=q.z(); result.orientation.w=q.w();
  return result;
}
}
GripperGeometry pandaGripperGeometry(const moveit::core::RobotModelConstPtr& model) {
  // Defaults are used only to measure fixed frame geometry, never as a planning start.
  moveit::core::RobotState geometry(model); geometry.setToDefaultValues();
  geometry.setVariablePosition("panda1_finger_joint1", 0.0);
  geometry.setVariablePosition("panda1_finger_joint2", 0.0); geometry.update();
  const auto hand = geometry.getGlobalLinkTransform("panda1_hand");
  const auto tcp = hand.inverse() * geometry.getGlobalLinkTransform("panda1_tcp");
  require(tcp.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-8), "Unsupported rotated Panda TCP");
  const auto* finger = model->getLinkModel("panda1_left_finger");
  require(finger != nullptr, "Missing Panda finger geometry");
  double volume = -1.0, pad = 0.0, half_height = 0.0;
  for (std::size_t i=0; i<finger->getShapes().size(); ++i) {
    if (finger->getShapes()[i]->type != shapes::BOX) continue;
    const auto* box = static_cast<const shapes::Box*>(finger->getShapes()[i].get());
    const double v = box->size[0]*box->size[1]*box->size[2];
    if (v <= volume) continue;
    const auto transform = hand.inverse() * geometry.getGlobalLinkTransform(finger) * finger->getCollisionOriginTransforms()[i];
    require(transform.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-8), "Unsupported finger pad frame");
    volume = v; pad = transform.translation().z(); half_height = box->size[2]/2.0;
  }
  require(volume > 0, "Missing main finger pad box");
  return {tcp.translation().z(), pad, half_height,
    model->getVariableBounds("panda1_finger_joint1").max_position_ +
    model->getVariableBounds("panda1_finger_joint2").max_position_};
}
GraspCandidate generateTopDownGrasp(const ObjectMetadata& object, const geometry_msgs::msg::Pose& object_pose,
                                   const GripperGeometry& geometry, const GraspParameters& parameters) {
  require(std::isfinite(geometry.hand_to_tcp) && std::isfinite(geometry.hand_to_pad) &&
    std::isfinite(geometry.pad_half_height) && std::isfinite(geometry.maximum_open_width) &&
    geometry.hand_to_tcp > 0 && geometry.hand_to_pad > 0 && geometry.pad_half_height > 0 &&
    geometry.maximum_open_width > 0, "Invalid gripper geometry");
  require(object.id == "cube" && object.collision_shape == CollisionShapeType::BOX,
          "Only the cube BOX baseline is supported");
  require(object.ownership == ObjectOwnership::WORLD, "Cube must remain in world");
  require(object.geometry.primitives.size() == 1, "Expected one cube box");
  const auto& dimensions = object.geometry.primitives[0].dimensions;
  require(dimensions.size() == 3 && dimensions[0] > 0 &&
    std::abs(dimensions[0]-dimensions[1]) < 1e-8 && std::abs(dimensions[0]-dimensions[2]) < 1e-8, "Expected cube dimensions");
  require(parameters.approach_direction.isApprox(-Eigen::Vector3d::UnitZ(), 1e-8), "Baseline approach must be world -Z");
  require(std::isfinite(parameters.pre_grasp_distance) && parameters.pre_grasp_distance > 0,
          "Invalid pre-grasp distance");
  require(std::isfinite(parameters.grasp_clearance) && parameters.grasp_clearance >= 0 &&
    parameters.grasp_clearance + geometry.pad_half_height < dimensions[2]/2, "Finger pad must overlap cube side below top");
  const auto& qmsg = object_pose.orientation;
  Eigen::Quaterniond q(qmsg.w,qmsg.x,qmsg.y,qmsg.z);
  require(q.coeffs().allFinite() && std::abs(q.norm()-1) <= 1e-6, "Invalid cube quaternion");
  require((q * Eigen::Vector3d::UnitZ()).dot(Eigen::Vector3d::UnitZ()) > std::cos(0.05), "Cube tilt exceeds top-down baseline");
  const double width = parameters.expected_cube_width == 0 ? dimensions[0] : parameters.expected_cube_width;
  require(std::isfinite(width) && std::abs(width-dimensions[0]) < 1e-8, "Width must match authoritative registry");
  const double opening = parameters.finger_open_width == 0 ? geometry.maximum_open_width : parameters.finger_open_width;
  require(std::isfinite(opening) && opening > width && opening <= geometry.maximum_open_width + 1e-8,
          "Open width must clear cube and respect gripper limits");
  require(std::set<std::string>(parameters.allowed_touch_links.begin(), parameters.allowed_touch_links.end()) ==
          std::set<std::string>{"panda1_left_finger", "panda1_right_finger"} && parameters.allowed_touch_links.size() == 2,
          "Only the two finger touch links are permitted");
  Eigen::Vector3d closing = q * Eigen::Vector3d::UnitX(); closing.z()=0; closing.normalize();
  const auto approach = parameters.approach_direction;
  Eigen::Matrix3d rotation;
  rotation.col(1)=closing; rotation.col(2)=approach; rotation.col(0)=closing.cross(approach);
  Eigen::Vector3d centre(object_pose.position.x, object_pose.position.y, object_pose.position.z);
  require(centre.allFinite(), "Invalid cube position");
  // Pad target = cube centre + clearance upward. Pad lies +Z from TCP.
  const auto grasp = centre - approach*(parameters.grasp_clearance + geometry.hand_to_pad - geometry.hand_to_tcp);
  GraspCandidate result;
  result.object_id=object.id; result.robot_id="panda1";
  result.grasp_pose=pose(grasp,rotation);
  result.pre_grasp_pose=pose(grasp-approach*parameters.pre_grasp_distance,rotation);
  result.approach_vector=approach; result.closing_axis=closing;
  result.pre_grasp_distance=parameters.pre_grasp_distance;
  result.expected_gripper_width=opening; result.expected_cube_width=width;
  result.allowed_touch_links=parameters.allowed_touch_links;
  return result;
}
bool pregraspJointNamesAllowed(const std::vector<std::string>& names) {
  std::set<std::string> expected={"panda1_rail_joint"};
  for(int i=1;i<=7;++i)expected.insert("panda1_joint"+std::to_string(i));
  return names.size()==8 && std::set<std::string>(names.begin(),names.end())==expected;
}
collision_detection::AllowedCollisionMatrix graspEvaluationACM(
    const collision_detection::AllowedCollisionMatrix& original, const GraspCandidate& candidate) {
  require(candidate.object_id == "cube" && candidate.robot_id == "panda1", "Unsupported touch policy target");
  require(std::set<std::string>(candidate.allowed_touch_links.begin(),candidate.allowed_touch_links.end()) ==
    std::set<std::string>{"panda1_left_finger", "panda1_right_finger"} && candidate.allowed_touch_links.size()==2,
    "Broad touch allowances are prohibited");
  auto copy = original;
  for (const auto& link : candidate.allowed_touch_links) copy.setEntry(link, candidate.object_id, true);
  return copy;
}
}
