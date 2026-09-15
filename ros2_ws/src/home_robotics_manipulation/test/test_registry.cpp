#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <set>
#include <filesystem>
#include "home_robotics_manipulation/object_registry.hpp"
namespace hm = home_robotics_manipulation;
std::string config_path() {
  return ament_index_cpp::get_package_share_directory("home_robotics_control") + "/config/objects.yaml";
}
TEST(Registry, ExactUniqueDeterministicIdsAndAuthoritativeGeometry) {
  hm::ObjectRegistry registry(config_path()), second(config_path());
  const auto source = YAML::LoadFile(config_path())["objects"];
  std::set<std::string> observed;
  for (const auto& [id, metadata] : registry.objects()) {
    observed.insert(id);
    EXPECT_EQ(metadata.mujoco_body, id);
    EXPECT_EQ(metadata.movable, source[id]["dynamic"].as<bool>());
    EXPECT_EQ(metadata.geometry, second.at(id).geometry);
    EXPECT_EQ(metadata.ownership, hm::ObjectOwnership::WORLD);
    const auto collision = source[id]["collision"];
    std::vector<YAML::Node> parts;
    if (collision["primitives"]) for (const auto& part : collision["primitives"]) parts.push_back(part);
    else parts.push_back(collision);
    ASSERT_EQ(metadata.geometry.primitives.size(), parts.size());
    for (std::size_t i = 0; i < parts.size(); ++i) {
      auto part = parts[i];
      auto primitive = metadata.geometry.primitives[i];
      if (part["type"].as<std::string>() == "box") {
        EXPECT_EQ(primitive.type, primitive.BOX);
        EXPECT_EQ(std::vector<double>(primitive.dimensions.begin(), primitive.dimensions.end()), part["dimensions"].as<std::vector<double>>());
      } else if (part["type"].as<std::string>() == "sphere") {
        EXPECT_EQ(primitive.type, primitive.SPHERE);
        EXPECT_DOUBLE_EQ(primitive.dimensions[0], part["radius"].as<double>());
      } else {
        EXPECT_EQ(primitive.type, primitive.CYLINDER);
        EXPECT_DOUBLE_EQ(primitive.dimensions[0], part["height"].as<double>());
        EXPECT_DOUBLE_EQ(primitive.dimensions[1], part["radius"].as<double>());
      }
      if (part["position"]) {
        const auto xyz = part["position"].as<std::vector<double>>();
        const auto pose = metadata.geometry.primitive_poses[i];
        EXPECT_DOUBLE_EQ(pose.position.x, xyz[0]);
        EXPECT_DOUBLE_EQ(pose.position.y, xyz[1]);
        EXPECT_DOUBLE_EQ(pose.position.z, xyz[2]);
        EXPECT_NEAR(pose.orientation.z, std::sin(part["yaw"].as<double>() / 2), 1e-12);
      }
    }
  }
  EXPECT_EQ(observed, (std::set<std::string>{"cube", "apple", "purple_ball", "bowl", "pan"}));
  EXPECT_EQ(registry.objects().size(), 5u);
}
TEST(Registry, PoseForwardingAndInvalidState) {
  hm::ObjectRegistry registry(config_path());
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world"; pose.pose.orientation.w = 1;
  pose.pose.position.x = 0.23;
  const auto collision = hm::world_object(registry.at("bowl"), pose);
  EXPECT_EQ(collision.pose, pose.pose);
  EXPECT_EQ(collision.operation, collision.ADD);
  EXPECT_EQ(collision.primitives.size(), 9u);
  EXPECT_EQ(collision.id, "bowl");
  pose.pose.orientation.w = 0;
  EXPECT_THROW(hm::world_object(registry.at("cube"), pose), std::runtime_error);
  pose.pose.orientation.w = 1; pose.header.frame_id = "map";
  EXPECT_THROW(hm::world_object(registry.at("cube"), pose), std::runtime_error);
}
TEST(Architecture, ExplicitLifecycleAndAttachmentOwnership) {
  using S = hm::ManipulationState;
  for (int i = int(S::IDLE); i < int(S::DONE); ++i) {
    EXPECT_TRUE(hm::transition_allowed(S(i), S(i + 1)));
    EXPECT_TRUE(hm::transition_allowed(S(i), S::FAILED));
  }
  EXPECT_FALSE(hm::transition_allowed(S::OBJECT_SELECTED, S::LIFTING));
  EXPECT_FALSE(hm::transition_allowed(S::GRIPPER_CLOSING, S::GRASP_VERIFIED));
  EXPECT_TRUE(hm::transition_allowed(S::FAILED, S::IDLE));
  EXPECT_TRUE(hm::transition_allowed(S::DONE, S::IDLE));
  EXPECT_FALSE(hm::world_sync_allowed(hm::ObjectOwnership::ATTACHED));
  EXPECT_TRUE(hm::world_sync_allowed(hm::ObjectOwnership::WORLD));
  const std::set<hm::ManipulationFailureReason> reasons = {
    hm::ManipulationFailureReason::OBJECT_NOT_FOUND, hm::ManipulationFailureReason::INVALID_OBJECT_STATE,
    hm::ManipulationFailureReason::IK_FAILED, hm::ManipulationFailureReason::PLANNING_FAILED,
    hm::ManipulationFailureReason::EXECUTION_FAILED, hm::ManipulationFailureReason::APPROACH_COLLISION,
    hm::ManipulationFailureReason::GRIPPER_FAILED, hm::ManipulationFailureReason::NO_CONTACT,
    hm::ManipulationFailureReason::GRASP_UNSTABLE, hm::ManipulationFailureReason::OBJECT_DROPPED,
    hm::ManipulationFailureReason::PLACE_FAILED, hm::ManipulationFailureReason::SCENE_SYNC_FAILED};
  EXPECT_EQ(reasons.size(), 12u);
  EXPECT_TRUE(std::filesystem::exists(ament_index_cpp::get_package_share_directory("home_robotics_manipulation") + "/launch/dynamic_object_scene_sync.launch.py"));
}

TEST(SceneSync, IdempotentDiffDoesNotClearStaticOrOverwriteAttached) {
  hm::ObjectRegistry registry(config_path());
  std::map<std::string, geometry_msgs::msg::PoseStamped> poses;
  for (const auto& [id, metadata] : registry.objects()) {
    poses[id].header.frame_id = "world";
    poses[id].pose.orientation.w = 1;
  }
  auto first = hm::world_update(registry, poses, {});
  auto again = hm::world_update(registry, poses, {});
  EXPECT_EQ(first, again);
  EXPECT_TRUE(first.is_diff);
  EXPECT_TRUE(first.robot_state.is_diff);
  EXPECT_TRUE(first.robot_state.attached_collision_objects.empty());
  EXPECT_EQ(first.world.collision_objects.size(), 5u);
  std::set<std::string> ids;
  for (const auto& object : first.world.collision_objects) {
    EXPECT_EQ(object.operation, object.ADD);
    EXPECT_TRUE(ids.insert(object.id).second);
    EXPECT_TRUE(registry.objects().count(object.id));
  }
  auto attached = hm::world_update(registry, poses, {"cube"});
  EXPECT_EQ(attached.world.collision_objects.size(), 4u);
  for (const auto& object : attached.world.collision_objects) EXPECT_NE(object.id, "cube");
}
