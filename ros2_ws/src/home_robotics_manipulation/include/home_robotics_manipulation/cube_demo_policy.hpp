#pragma once
#include <cmath>
#include <stdexcept>
#include <moveit_msgs/msg/planning_scene.hpp>
namespace home_robotics_manipulation {
struct DemoReadiness {
  bool joints_fresh=false, cube_fresh=false, scene_synced=false, arm_active=false,
       gripper_active=false, static_loaded=false, five_objects=false,
       cube_world_unique=false, full_state_valid=false;
  double rail_separation=0;
  bool ready() const {
    return joints_fresh&&cube_fresh&&scene_synced&&arm_active&&gripper_active&&static_loaded&&
      five_objects&&cube_world_unique&&full_state_valid&&std::isfinite(rail_separation)&&rail_separation>=0.7;
  }
};
inline bool physicalCommandsAllowed(bool execute, const DemoReadiness& gate) { return execute&&gate.ready(); }
inline moveit_msgs::msg::PlanningScene attachCube(const moveit_msgs::msg::AttachedCollisionObject& attached,
                                                 bool verified, bool stabilized) {
  if(!verified||!stabilized||(attached.object.id!="cube"&&attached.object.id!="purple_ball")||attached.link_name!="panda1_hand")
    throw std::runtime_error("GRASP_UNSTABLE: verified stabilization required for attachment");
  moveit_msgs::msg::PlanningScene diff;diff.is_diff=true;diff.robot_state.is_diff=true;
  moveit_msgs::msg::CollisionObject remove;remove.id=attached.object.id;remove.operation=remove.REMOVE;
  // MoveIt atomically removes the world object when applying the attachment.
  diff.robot_state.attached_collision_objects.push_back(attached);return diff;
}
inline moveit_msgs::msg::PlanningScene releaseCube(const moveit_msgs::msg::CollisionObject& world,
                                                  bool supported, bool stabilized) {
  if(!supported||stabilized||(world.id!="cube"&&world.id!="purple_ball"))throw std::runtime_error("PLACE_FAILED: supported release required");
  moveit_msgs::msg::PlanningScene diff;diff.is_diff=true;diff.robot_state.is_diff=true;
  moveit_msgs::msg::AttachedCollisionObject remove;remove.link_name="panda1_hand";remove.object.id=world.id;remove.object.operation=remove.object.REMOVE;
  diff.robot_state.attached_collision_objects.push_back(remove);diff.world.collision_objects.push_back(world);return diff;
}
}
