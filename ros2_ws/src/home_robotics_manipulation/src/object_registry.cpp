#include "home_robotics_manipulation/object_registry.hpp"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <shape_msgs/msg/solid_primitive.hpp>
namespace home_robotics_manipulation {
ObjectRegistry::ObjectRegistry(const std::string& path) {
  const auto document = YAML::LoadFile(path)["objects"];
  if (!document.IsMap()) throw std::runtime_error("objects must be a mapping");
  for (const auto& entry : document) {
    const auto id = entry.first.as<std::string>();
    const auto node = entry.second;
    if (!node["planning_scene_enabled"].as<bool>()) continue;
    if (node["id"].as<std::string>() != id) throw std::runtime_error("Object key/id mismatch");
    ObjectMetadata metadata;
    metadata.id = id;
    metadata.mujoco_body = id;  // SceneBuilder uses the same config key as body name.
    metadata.movable = node["dynamic"].as<bool>();
    const auto category = node["category"].as<std::string>();
    if (category == "manipulable") metadata.type = ObjectType::MANIPULABLE;
    else if (category == "container") metadata.type = ObjectType::CONTAINER;
    else if (category == "future_manipulable") metadata.type = ObjectType::FUTURE_MANIPULABLE;
    else throw std::runtime_error("Unknown object category");
    if (node["grasp_profile"] && !node["grasp_profile"].IsNull())
      metadata.grasp_profile_reference = node["grasp_profile"].as<std::string>();
    const auto collision = node["collision"];
    const auto type = collision["type"].as<std::string>();
    if (type == "box") metadata.collision_shape = CollisionShapeType::BOX;
    else if (type == "sphere") metadata.collision_shape = CollisionShapeType::SPHERE;
    else if (type == "primitive_compound") metadata.collision_shape = CollisionShapeType::PRIMITIVE_COMPOUND;
    else throw std::runtime_error("Unsupported collision shape");
    std::vector<YAML::Node> parts;
    if (type == "primitive_compound") for (const auto& part : collision["primitives"]) parts.push_back(part);
    else parts.push_back(collision);
    if (parts.empty()) throw std::runtime_error("Empty collision geometry");
    for (const auto& part : parts) {
      shape_msgs::msg::SolidPrimitive shape;
      const auto kind = part["type"].as<std::string>();
      if (kind == "box") {
        shape.type = shape.BOX;
        const auto dimensions = part["dimensions"].as<std::vector<double>>();
        shape.dimensions.assign(dimensions.begin(), dimensions.end());
        if (shape.dimensions.size() != 3) throw std::runtime_error("BOX needs 3 dimensions");
      } else if (kind == "sphere") {
        shape.type = shape.SPHERE;
        shape.dimensions = {part["radius"].as<double>()};
      } else if (kind == "cylinder") {
        shape.type = shape.CYLINDER;
        shape.dimensions = {part["height"].as<double>(), part["radius"].as<double>()};
      } else throw std::runtime_error("Unsupported primitive");
      for (double value : shape.dimensions)
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Invalid dimension");
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1;
      if (part["position"]) {
        const auto xyz = part["position"].as<std::vector<double>>();
        pose.position.x = xyz.at(0); pose.position.y = xyz.at(1); pose.position.z = xyz.at(2);
      }
      if (part["yaw"]) {
        const double yaw = part["yaw"].as<double>();
        pose.orientation.w = std::cos(yaw / 2); pose.orientation.z = std::sin(yaw / 2);
      }
      metadata.geometry.primitives.push_back(shape);
      metadata.geometry.primitive_poses.push_back(pose);
    }
    metadata.geometry.id = id;
    if (!objects_.emplace(id, metadata).second) throw std::runtime_error("Duplicate object ID");
  }
}
moveit_msgs::msg::CollisionObject world_object(const ObjectMetadata& metadata,
                                               const geometry_msgs::msg::PoseStamped& pose) {
  if (pose.header.frame_id != "world") throw std::runtime_error("Object pose frame must be world");
  const auto& q = pose.pose.orientation;
  const auto& t = pose.pose.position;
  const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-6)
    throw std::runtime_error("Invalid object position/quaternion");
  auto object = metadata.geometry;
  object.header = pose.header;
  object.pose = pose.pose;
  object.operation = object.ADD;
  return object;
}
moveit_msgs::msg::PlanningScene world_update(const ObjectRegistry& registry,
    const std::map<std::string, geometry_msgs::msg::PoseStamped>& poses,
    const std::set<std::string>& attached_ids) {
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  for (const auto& [id, metadata] : registry.objects()) {
    if (!world_sync_allowed(metadata.ownership) || attached_ids.count(id)) continue;
    diff.world.collision_objects.push_back(world_object(metadata, poses.at(id)));
  }
  return diff;
}

}
