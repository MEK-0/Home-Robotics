#pragma once
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "home_robotics_manipulation/manipulation_state.hpp"
namespace home_robotics_manipulation {
enum class ObjectType { MANIPULABLE, CONTAINER, FUTURE_MANIPULABLE };
enum class CollisionShapeType { BOX, SPHERE, PRIMITIVE_COMPOUND };
enum class ObjectOwnership { WORLD, ATTACHED };
struct ObjectMetadata {
  std::string id;
  ObjectType type;
  CollisionShapeType collision_shape;
  std::string mujoco_body;
  bool movable;
  ObjectOwnership ownership = ObjectOwnership::WORLD;
  std::string grasp_profile_reference;  // Reserved metadata, no grasp generation.
  moveit_msgs::msg::CollisionObject geometry;  // Local primitives and local poses.
};
struct ObjectPoseState {
  std::string id;
  geometry_msgs::msg::PoseStamped pose;
  ObjectOwnership ownership = ObjectOwnership::WORLD;
};
inline bool world_sync_allowed(ObjectOwnership owner) { return owner == ObjectOwnership::WORLD; }
}
