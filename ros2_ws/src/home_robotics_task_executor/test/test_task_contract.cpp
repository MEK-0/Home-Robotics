#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "home_robotics_interfaces/action/pick_and_place.hpp"
#include "home_robotics_task_executor/task_types.hpp"

namespace hrt = home_robotics_task_executor;

TEST(TaskContract, PickAndPlaceActionAndStateVocabularyAreStable) {
  home_robotics_interfaces::action::PickAndPlace::Goal goal;
  goal.object_id = "cube";
  goal.target_id = "surface_left_2";
  EXPECT_EQ(goal.object_id, "cube");
  EXPECT_EQ(goal.target_id, "surface_left_2");
  EXPECT_EQ(static_cast<int>(hrt::TaskState::CANCELED), 13);
  EXPECT_EQ(static_cast<int>(hrt::TaskFailureReason::CANCELED), 12);
  EXPECT_TRUE(hrt::TaskRequest{}.object_id.empty());
}

TEST(TaskContract, AuthoritativeObjectAndTargetResolution) {
  const auto config = ament_index_cpp::get_package_share_directory("home_robotics_control") + "/config/";
  const auto objects = YAML::LoadFile(config + "objects.yaml")["objects"];
  EXPECT_TRUE(objects["cube"]["dynamic"].as<bool>());
  EXPECT_FALSE(objects["does_not_exist"]);
  const auto scene = YAML::LoadFile(config + "scene.yaml");
  EXPECT_TRUE(scene["scene"]["surfaces"]["surface_left_2"]);
  EXPECT_FALSE(scene["scene"]["surfaces"]["does_not_exist"]);
  EXPECT_EQ(objects["cube"]["initial"]["support_surface"].as<std::string>(), "surface_left_1");
}
