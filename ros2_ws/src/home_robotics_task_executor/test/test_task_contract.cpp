#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iterator>
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


namespace {
std::string read_file(const std::filesystem::path & path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
std::filesystem::path package_root() {
  return std::filesystem::path(__FILE__).parent_path().parent_path();
}
}  // namespace

TEST(TaskArchitecture, ExecutorHasNoPhysicsOrRawMoveItOwnership) {
  const auto root = package_root();
  const std::string source = read_file(root / "src/task_executor.cpp") +
    read_file(root / "src/manipulation_adapter.cpp") +
    read_file(root / "include/home_robotics_task_executor/task_executor.hpp");
  for (const auto & forbidden : {"mujoco.MjModel", "mujoco.MjData", "MjModel", "MjData",
      "MoveGroupInterface", "setFromIK", "PlanningScene", "RRTConnect"}) {
    EXPECT_EQ(source.find(forbidden), std::string::npos) << forbidden;
  }
}

TEST(TaskArchitecture, DependencyDirectionAndSafeDefaultsAreStable) {
  const auto root = package_root();
  const std::string package = read_file(root / "package.xml");
  const std::string cmake = read_file(root / "CMakeLists.txt");
  const std::string launch = read_file(root / "launch/task_executor.launch.py");
  EXPECT_NE(package.find("<depend>home_robotics_manipulation</depend>"), std::string::npos);
  EXPECT_NE(cmake.find("find_package(home_robotics_manipulation REQUIRED)"), std::string::npos);
  EXPECT_NE(launch.find("DeclareLaunchArgument(\"execute\", default_value=\"false\")"), std::string::npos);
  const auto manipulation_root = root.parent_path() / "home_robotics_manipulation";
  EXPECT_EQ(read_file(manipulation_root / "package.xml").find("home_robotics_task_executor"), std::string::npos);
  EXPECT_NE(read_file(manipulation_root / "CMakeLists.txt").find("cube_pick_place_demo"), std::string::npos);
}
