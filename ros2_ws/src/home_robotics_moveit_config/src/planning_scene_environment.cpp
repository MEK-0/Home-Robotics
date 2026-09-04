#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace
{
shape_msgs::msg::SolidPrimitive box(const std::vector<double>& dimensions)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(dimensions.begin(), dimensions.end());
  return primitive;
}

geometry_msgs::msg::Pose pose(
  double x, double y, double z, const std::vector<double>& quaternion = {1.0, 0.0, 0.0, 0.0})
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.position.z = z;
  result.orientation.w = quaternion.at(0);
  result.orientation.x = quaternion.at(1);
  result.orientation.y = quaternion.at(2);
  result.orientation.z = quaternion.at(3);
  return result;
}

moveit_msgs::msg::CollisionObject object(const std::string& id, const std::string& frame)
{
  moveit_msgs::msg::CollisionObject result;
  result.id = id;
  result.header.frame_id = frame;
  result.operation = moveit_msgs::msg::CollisionObject::ADD;
  return result;
}

std::vector<double> values(const YAML::Node& node)
{
  return node.as<std::vector<double>>();
}

std::vector<moveit_msgs::msg::CollisionObject> build_environment(const YAML::Node& scene)
{
  const std::string frame = scene["frame"].as<std::string>();
  if (frame != "world") {
    throw std::runtime_error("Authoritative scene frame must be world");
  }
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  const auto floor = scene["floor"];
  const auto floor_position = values(floor["pose"]["position"]);
  const auto border = floor["arena_border"];
  const auto border_dimensions = values(border["dimensions"]);
  const double length = border_dimensions.at(0);
  const double width = border_dimensions.at(1);
  const double strip = border["width"].as<double>();
  const double border_height = border["height"].as<double>();
  const double border_z = floor_position.at(2) + border["floor_clearance"].as<double>() + border_height / 2.0;

  auto floor_object = object(floor["id"].as<std::string>(), frame);
  floor_object.primitives.push_back(box({length, width, 0.10}));
  floor_object.primitive_poses.push_back(pose(floor_position.at(0), floor_position.at(1), floor_position.at(2) - 0.05));
  objects.push_back(floor_object);

  struct BorderDefinition { std::string id; std::vector<double> dimensions; geometry_msgs::msg::Pose pose; };
  const std::vector<BorderDefinition> borders = {
    {"arena_front", {strip, width, border_height}, pose(floor_position.at(0) + length / 2.0 - strip / 2.0, floor_position.at(1), border_z)},
    {"arena_back", {strip, width, border_height}, pose(floor_position.at(0) - length / 2.0 + strip / 2.0, floor_position.at(1), border_z)},
    {"arena_left", {length - 2.0 * strip, strip, border_height}, pose(floor_position.at(0), floor_position.at(1) + width / 2.0 - strip / 2.0, border_z)},
    {"arena_right", {length - 2.0 * strip, strip, border_height}, pose(floor_position.at(0), floor_position.at(1) - width / 2.0 + strip / 2.0, border_z)},
  };
  for (const auto& definition : borders) {
    auto collision = object(definition.id, frame);
    collision.primitives.push_back(box(definition.dimensions));
    collision.primitive_poses.push_back(definition.pose);
    objects.push_back(collision);
  }

  const auto table_geometry = scene["table_geometry"];
  const auto leg_dimensions = values(table_geometry["leg_dimensions"]);
  const auto inset = values(table_geometry["leg_center_inset"]);
  for (const auto& entry : scene["surfaces"]) {
    const auto surface = entry.second;
    const std::string id = surface["id"].as<std::string>();
    const auto center = values(surface["pose"]["position"]);
    const auto quaternion = values(surface["pose"]["quaternion_wxyz"]);
    const auto dimensions = values(surface["dimensions"]);
    auto collision = object(id, frame);
    collision.primitives.push_back(box(dimensions));
    collision.primitive_poses.push_back(pose(center.at(0), center.at(1), center.at(2) - dimensions.at(2) / 2.0, quaternion));
    const double leg_z = floor_position.at(2) + leg_dimensions.at(2) / 2.0;
    const double leg_x = dimensions.at(0) / 2.0 - inset.at(0);
    const double leg_y = dimensions.at(1) / 2.0 - inset.at(1);
    for (const auto& offset : std::vector<std::pair<double, double>>{{leg_x, leg_y}, {leg_x, -leg_y}, {-leg_x, leg_y}, {-leg_x, -leg_y}}) {
      collision.primitives.push_back(box(leg_dimensions));
      collision.primitive_poses.push_back(pose(center.at(0) + offset.first, center.at(1) + offset.second, leg_z, quaternion));
    }
    objects.push_back(collision);
  }

  const auto rail = scene["shared_rail"];
  const auto rail_position = values(rail["pose"]["position"]);
  const auto supports = rail["supports"];
  const auto support_dimensions = values(supports["dimensions"]);
  for (std::size_t index = 0; index < supports["names"].size(); ++index) {
    const auto offset = values(supports["positions"][index]);
    auto collision = object(supports["names"][index].as<std::string>(), frame);
    collision.primitives.push_back(box(support_dimensions));
    collision.primitive_poses.push_back(pose(rail_position.at(0) + offset.at(0), rail_position.at(1) + offset.at(1), rail_position.at(2) + offset.at(2)));
    objects.push_back(collision);
  }
  return objects;
}
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("planning_scene_environment");
  const auto default_path = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("home_robotics_moveit_config")) / "config" / "scene.yaml";
  const std::string scene_config = node->declare_parameter("scene_config", default_path.string());
  const double wait_timeout = node->declare_parameter("wait_timeout", 10.0);
  try {
    const YAML::Node document = YAML::LoadFile(scene_config);
    const auto objects = build_environment(document["scene"]);
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_timeout);
    bool applied = false;
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      if (planning_scene.applyCollisionObjects(objects)) {
        applied = true;
        break;
      }
      std::this_thread::sleep_for(200ms);
    }
    if (!applied) {
      throw std::runtime_error("PlanningScene did not accept static collision objects before timeout");
    }
    std::set<std::string> expected;
    for (const auto& item : objects) expected.insert(item.id);
    std::map<std::string, moveit_msgs::msg::CollisionObject> observed;
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      observed = planning_scene.getObjects(std::vector<std::string>(expected.begin(), expected.end()));
      if (observed.size() == expected.size()) break;
      std::this_thread::sleep_for(200ms);
    }
    if (observed.size() != expected.size()) {
      throw std::runtime_error("PlanningScene object verification timed out");
    }
    const auto all_objects = planning_scene.getObjects();
    RCLCPP_INFO(node->get_logger(), "Loaded and verified %zu static collision objects in frame 'world' (PlanningScene total=%zu)", objects.size(), all_objects.size());
    for (const auto& item : objects) RCLCPP_INFO(node->get_logger(), "  %s", item.id.c_str());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Static environment loading failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
