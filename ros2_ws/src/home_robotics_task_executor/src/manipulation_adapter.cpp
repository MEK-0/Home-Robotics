#include "home_robotics_task_executor/manipulation_adapter.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <signal.h>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace home_robotics_task_executor {
namespace {
std::string shell_quote(const std::string & value) {
  std::string quoted{"'"};
  for (const char c : value) quoted += c == '\'' ? "'\\\"'\\\"'" : std::string(1, c);
  return quoted + "'";
}
geometry_msgs::msg::Pose parse_pose(const YAML::Node & node) {
  geometry_msgs::msg::Pose pose;
  if (!node || !node.IsSequence() || node.size() != 7) return pose;
  pose.position.x = node[0].as<double>(); pose.position.y = node[1].as<double>(); pose.position.z = node[2].as<double>();
  pose.orientation.x = node[3].as<double>(); pose.orientation.y = node[4].as<double>(); pose.orientation.z = node[5].as<double>(); pose.orientation.w = node[6].as<double>();
  return pose;
}
}  // namespace

ManipulationResult ManipulationAdapter::pick_and_place(const TaskRequest & request, bool execute,
  const std::function<bool()> & cancellation_requested) const {
  ManipulationResult result;
  if (cancellation_requested()) {
    result.failure_reason = "CANCELED";
    result.message = "Canceled at the safe boundary before Phase 4 starts";
    return result;
  }
  const auto report_path = std::filesystem::temp_directory_path() /
    ("home_robotics_" + std::to_string(::getpid()) + "_" + request.object_id + ".yaml");
  const std::string command = "ros2 launch home_robotics_manipulation cube_pick_place_demo.launch.py"
    " robot:=panda1 object:=" + shell_quote(request.object_id) +
    " target:=" + shell_quote(request.target_id) + " execute:=" + (execute ? "true" : "false") +
    " result_file:=" + shell_quote(report_path.string());
  RCLCPP_INFO(logger_, "Delegating task to Phase 4 manipulation pipeline (execute=%s)", execute ? "true" : "false");
  const pid_t child = ::fork();
  if (child == -1) {
    result.failure_reason = "EXECUTION_FAILED";
    result.message = "Could not start the Phase 4 manipulation pipeline";
    return result;
  }
  if (child == 0) {
    ::setpgid(0, 0);
    ::execl("/bin/sh", "sh", "-c", command.c_str(), static_cast<char *>(nullptr));
    ::_exit(127);
  }
  int status = 0;
  while (::waitpid(child, &status, WNOHANG) == 0) {
    if (!execute && cancellation_requested()) {
      ::kill(-child, SIGTERM);
      (void)::waitpid(child, &status, 0);
      result.failure_reason = "CANCELED";
      result.message = "Planning-only task canceled before physical execution";
      std::error_code ignored;
      std::filesystem::remove(report_path, ignored);
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  try {
    const auto report = YAML::LoadFile(report_path.string());
    result.success = report["success"].as<bool>(false) && WIFEXITED(status) && WEXITSTATUS(status) == 0;
    result.failure_reason = report["failure_reason"].as<std::string>(result.success ? "NONE" : "INVALID_OBJECT_STATE");
    result.message = report["failure_detail"].as<std::string>(result.success ?
      (execute ? "Phase 4 placement verified" : "Planning-only preflight passed; no physical commands issued") :
      "Phase 4 manipulation failed");
    result.final_object_pose = parse_pose(report["final_pose"]);
  } catch (const std::exception & error) {
    result.failure_reason = "INVALID_OBJECT_STATE";
    result.message = "Phase 4 pipeline produced no readable result: " + std::string(error.what());
  }
  std::error_code ignored;
  std::filesystem::remove(report_path, ignored);
  return result;
}
}  // namespace home_robotics_task_executor~se
