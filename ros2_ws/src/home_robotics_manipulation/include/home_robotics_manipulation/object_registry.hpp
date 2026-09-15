#pragma once
#include <map>
#include <set>
#include <moveit_msgs/msg/planning_scene.hpp>
#include "home_robotics_manipulation/manipulation_types.hpp"
namespace home_robotics_manipulation {
// Read-only typed view of the existing config/objects.yaml, not another registry source.
class ObjectRegistry {
public:
  explicit ObjectRegistry(const std::string& config_path);
  const std::map<std::string, ObjectMetadata>& objects() const { return objects_; }
  const ObjectMetadata& at(const std::string& id) const { return objects_.at(id); }
private:
  std::map<std::string, ObjectMetadata> objects_;
};
moveit_msgs::msg::CollisionObject world_object(const ObjectMetadata&, const geometry_msgs::msg::PoseStamped&);
moveit_msgs::msg::PlanningScene world_update(const ObjectRegistry&,
  const std::map<std::string, geometry_msgs::msg::PoseStamped>&,
  const std::set<std::string>& attached_ids);
}
