#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <set>

namespace
{
std::string values_string(const std::vector<std::string>& names, const std::vector<double>& values)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(12);
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << names[index] << '=' << values[index];
  }
  return stream.str();
}

void apply_joint_state(
  moveit::core::RobotState& state, const sensor_msgs::msg::JointState& message)
{
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < message.name.size(); ++index) {
    positions.emplace(message.name[index], message.position[index]);
  }
  state.setVariablePositions(positions);
  state.update();
}

std::vector<double> read_group_values(
  const moveit::core::RobotState& state, const std::vector<std::string>& joints)
{
  std::vector<double> values;
  values.reserve(joints.size());
  for (const auto& joint : joints) {
    values.push_back(state.getVariablePosition(joint));
  }
  return values;
}

moveit_msgs::srv::GetStateValidity::Response::SharedPtr check_validity(
  const rclcpp::Node::SharedPtr& node,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr& client,
  const moveit::core::RobotState& state, const std::string& group)
{
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  moveit::core::robotStateToRobotStateMsg(state, request->robot_state);
  request->group_name = group;
  if (!client->wait_for_service(10s)) {
    RCLCPP_ERROR(node->get_logger(), "State-validity service is unavailable");
    return nullptr;
  }
  auto future = client->async_send_request(request);
  if (future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "State-validity request timed out");
    return nullptr;
  }
  return future.get();
}
bool cross(const std::string& a, const std::string& b)
{
  return (a.rfind("panda1_", 0) == 0 && b.rfind("panda2_", 0) == 0) ||
         (b.rfind("panda1_", 0) == 0 && a.rfind("panda2_", 0) == 0);
}
struct Check { bool valid; std::size_t cross_contacts; };
Check inspect(planning_scene::PlanningScene& scene, moveit::core::RobotState& state,
              const rclcpp::Logger& logger, const std::string& label, bool verbose = true,
              const std::string& group = "", double bounds_tolerance = 1e-6)
{
  state.update();
  collision_detection::CollisionRequest request;
  request.group_name = group;
  request.contacts = true;
  request.max_contacts = 10000;
  request.max_contacts_per_pair = 1;
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state);
  std::size_t count = 0;
  for (const auto& pair : result.contacts) {
    if (cross(pair.first.first, pair.first.second)) ++count;
    if (verbose) RCLCPP_INFO(logger, "%s contact: %s <-> %s", label.c_str(),
      pair.first.first.c_str(), pair.first.second.c_str());
  }
  const bool valid = state.satisfiesBounds(bounds_tolerance) && !result.collision;
  if (verbose) RCLCPP_INFO(logger, "%s validity=%s bounds=%s cross_pairs=%zu", label.c_str(),
    valid ? "VALID" : "INVALID", state.satisfiesBounds(bounds_tolerance) ? "OK(tolerance=1e-6)" : "FAIL", count);
  return {valid, count};
}
double separation(const moveit::core::RobotState& state)
{
  return state.getVariablePosition("panda2_rail_joint") - state.getVariablePosition("panda1_rail_joint");
}
void require(bool condition, const std::string& reason)
{
  if (!condition) throw std::runtime_error(reason);
}
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dual_robot_collision_validation",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  std::string mode;
  bool execute = false;
  node->get_parameter_or<std::string>("mode", mode, "current");
  node->get_parameter_or("execute", execute, false);
  const std::set<std::string> modes = {"current", "safe_panda1", "safe_panda2",
    "cross_collision", "path_collision", "reverse_path_collision"};
  const bool safe_mode = mode.rfind("safe_", 0) == 0;
  if (!modes.count(mode) || (execute && !safe_mode)) {
    RCLCPP_ERROR(node->get_logger(), "VALIDATION FAIL: %s", !modes.count(mode) ? "Unknown mode" : "execute=true is permitted only for safe modes");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });
  int status = 1;
  try {
    const std::set<std::string> modes = {"current", "safe_panda1", "safe_panda2",
      "cross_collision", "path_collision", "reverse_path_collision"};
    require(modes.count(mode), "Unknown mode");
    // Bounds comparisons tolerate 1e-6 for measured finger-limit roundoff.
    // No geometry, model limit, ACM entry or hardware rule is changed.
    const bool safe = mode.rfind("safe_", 0) == 0;
    require(!execute || safe, "execute=true is permitted only for safe modes");
    const std::string active = (mode == "safe_panda2" || mode == "reverse_path_collision") ? "panda2" : "panda1";
    const std::string stationary = active == "panda1" ? "panda2" : "panda1";
    const std::string group = active + "_manipulator";
    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "mode=%s active=%s stationary=%s execute=%s", mode.c_str(),
      group.c_str(), stationary.c_str(), execute ? "true" : "false");
    moveit::planning_interface::MoveGroupInterface move_group(node, group);
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    auto current = move_group.getCurrentState(10.0);
    require(bool(current), "No current state");
    auto reader = rclcpp::Node::make_shared("dual_collision_state_reader");
    sensor_msgs::msg::JointState message;
    require(rclcpp::wait_for_message(message, reader, "/joint_states", 10s), "No fresh joint state");
    apply_joint_state(*current, message);
    auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    require(client->wait_for_service(10s), "No planning scene service");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 1023;
    auto future = client->async_send_request(request);
    require(future.wait_for(10s) == std::future_status::ready, "Scene request timeout");
    planning_scene::PlanningScene scene(move_group.getRobotModel());
    scene.setPlanningSceneMsg(future.get()->scene);
    require(scene.getWorld()->size() == 14, "Expected 14 static environment objects");
    std::size_t checked = 0;
    for (const auto& a : current->getRobotModel()->getLinkModelNames()) {
      for (const auto& b : current->getRobotModel()->getLinkModelNames()) {
        if (a.rfind("panda1_", 0) != 0 || b.rfind("panda2_", 0) != 0) continue;
        collision_detection::AllowedCollision::Type type;
        const bool found = scene.getAllowedCollisionMatrix().getAllowedCollision(a, b, type);
        require(!found || type == collision_detection::AllowedCollision::NEVER,
          "Runtime ACM allows cross-robot pair: " + a + " / " + b);
        ++checked;
      }
    }
    RCLCPP_INFO(logger, "Runtime ACM: %zu cross-robot pairs enabled", checked);
    RCLCPP_INFO(logger, "Start rails: panda1=%.8f panda2=%.8f separation=%.8f minimum=0.70000000",
      current->getVariablePosition("panda1_rail_joint"), current->getVariablePosition("panda2_rail_joint"), separation(*current));
    require(inspect(scene, *current, logger, "current full state").valid, "Invalid current state");
    require(separation(*current) >= 0.7, "Current rail separation unsafe");
    auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
    auto remote = check_validity(node, validity_client, *current, "");
    require(remote && remote->valid, "Server current full state invalid");
    auto target = *current;
    const auto* jmg = current->getJointModelGroup(group);
    const auto names = jmg->getVariableNames();
    std::vector<std::string> stationary_names;
    for (const auto& name : current->getVariableNames())
      if (name.rfind(stationary + "_", 0) == 0) stationary_names.push_back(name);
    auto observed_drift = std::make_shared<std::atomic<double>>(0.0);
    auto baseline = std::make_shared<moveit::core::RobotState>(*current);
    auto drift_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [baseline, stationary_names, observed_drift](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          if (std::find(stationary_names.begin(), stationary_names.end(), msg->name[i]) == stationary_names.end()) continue;
          const double delta = std::abs(msg->position[i] - baseline->getVariablePosition(msg->name[i]));
          double old = observed_drift->load();
          while (delta > old && !observed_drift->compare_exchange_weak(old, delta)) {}
        }
      });
    if (mode == "cross_collision") {
      target.setJointGroupPositions("panda1_manipulator",
        std::vector<double>{0.0, 0.10, -0.10, 0.10, -1.50, 0.05, 1.50, -0.70});
      target.setJointGroupPositions("panda2_manipulator",
        std::vector<double>{0.0, -0.10, 0.10, -0.10, -1.50, -0.05, 1.50, -0.70});
      for (const auto& name : target.getVariableNames())
        if (name.find("finger_joint") != std::string::npos) target.setVariablePosition(name, 0.04);
      RCLCPP_INFO(logger, "OFFLINE ONLY: both rails=0, deterministic arm targets and open fingers; separation=0 < 0.7; never executed");
      RCLCPP_INFO(logger, "Exact state: %s", values_string(target.getVariableNames(),
        read_group_values(target, target.getVariableNames())).c_str());
      for (const auto& scope : {std::string(""), std::string("panda1_manipulator"), std::string("panda2_manipulator")}) {
        auto check = inspect(scene, target, logger, "overlap scope=" + scope, true, scope);
        require(!check.valid && check.cross_contacts > 0, "Missing cross-robot collision");
        remote = check_validity(node, validity_client, target, scope);
        require(remote && !remote->valid, "Server failed to reject overlap");
      }
    } else if (mode != "current") {
      if (safe) {
        const std::vector<double> values = active == "panda1" ?
          std::vector<double>{-0.85, 0.10, -0.10, 0.10, -1.50, 0.05, 1.50, -0.70} :
          std::vector<double>{0.85, -0.10, 0.10, -0.10, -1.50, -0.05, 1.50, -0.70};
        target.setJointGroupPositions(jmg, values);
      } else {
        current->setVariablePosition(stationary + "_rail_joint", active == "panda1" ? -0.30 : 0.30);
        current->update();
        require(inspect(scene, *current, logger, "offline path start").valid, "Invalid offline path start");
        target = *current;
        RCLCPP_INFO(logger, "Offline path start full state: %s separation=%.9f",
          values_string(current->getVariableNames(), read_group_values(*current, current->getVariableNames())).c_str(), separation(*current));
        RCLCPP_INFO(logger, "Offline stationary rail=%f; remaining stationary joints retain measured values", current->getVariablePosition(stationary + "_rail_joint"));
        // Valid goal on the far side of the stationary carriage. Rail ordering is
        // deliberately infeasible: this entire scenario is offline and plan-only.
        const auto& bounds = current->getRobotModel()->getVariableBounds(active + "_rail_joint");
        target.setVariablePosition(active + "_rail_joint", active == "panda1" ?
          bounds.max_position_ - 0.01 : bounds.min_position_ + 0.01);
        bool blocked = false;
        for (int i = 0; i <= 400; ++i) {
          auto sample = *current;
          current->interpolate(target, i / 400.0, sample);
          auto check = inspect(scene, sample, logger, "direct route", false);
          if (check.cross_contacts) {
            inspect(scene, sample, logger, "direct route cross collision");
            blocked = true;
            break;
          }
        }
        require(blocked, "Path scenario does not encounter stationary robot");
        RCLCPP_INFO(logger, "OFFLINE path scenario: rail ordering safety may be violated; execution prohibited");
      }
      RCLCPP_INFO(logger, "Target: %s", values_string(names, read_group_values(target, names)).c_str());
      require(inspect(scene, target, logger, "target full state").valid, "Target must be geometrically valid");
      remote = check_validity(node, validity_client, target, "");
      require(remote && remote->valid, "Server target invalid");
      move_group.setStartState(*current);
      require(move_group.setJointValueTarget(target), "Target rejected");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_start = std::chrono::steady_clock::now();
      const auto code = move_group.plan(plan);
      RCLCPP_INFO(logger, "Planning wall time=%.6f s", std::chrono::duration<double>(std::chrono::steady_clock::now() - plan_start).count());
      const bool planned = code == moveit::core::MoveItErrorCode::SUCCESS;
      const auto& trajectory = plan.trajectory.joint_trajectory;
      RCLCPP_INFO(logger, "Planning=%s code=%d time=%.6f points=%zu", planned ? "SUCCESS" : "FAILURE",
        code.val, plan.planning_time, trajectory.points.size());
      require(planned || !safe, "Safe planning failed");
      if (!planned) require(code.val == moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED ||
        code.val == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT ||
        code.val == moveit_msgs::msg::MoveItErrorCodes::FAILURE,
        "Path failure is not a planner failure (check service/start/goal)");
      if (planned) {
        require(!trajectory.points.empty(), "Empty successful trajectory");
        for (const auto& name : trajectory.joint_names)
          require(std::find(names.begin(), names.end(), name) != names.end(), "Trajectory commands another group");
        auto previous = *current;
        std::size_t samples = 0;
        for (const auto& point : trajectory.points) {
          require(point.positions.size() == trajectory.joint_names.size(), "Malformed trajectory");
          auto waypoint = *current;
          for (std::size_t i = 0; i < point.positions.size(); ++i)
            waypoint.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
          double delta = 0;
          for (const auto& name : names) delta = std::max(delta,
            std::abs(waypoint.getVariablePosition(name) - previous.getVariablePosition(name)));
          const int steps = std::max(1, static_cast<int>(std::ceil(delta / 0.005)));
          for (int i = 1; i <= steps; ++i) {
            auto sample = previous;
            previous.interpolate(waypoint, double(i) / steps, sample);
            auto check = inspect(scene, sample, logger, "trajectory", false);
            if (!check.valid) inspect(scene, sample, logger, "INVALID trajectory");
            require(check.valid, "Generated trajectory contains collision or out-of-bounds state");
            if (safe) require(separation(sample) >= 0.7, "Trajectory violates rail safety");
            ++samples;
          }
          previous = waypoint;
        }
        RCLCPP_INFO(logger, "Waypoint and interpolated validation: PASS samples=%zu cross_contacts=0", samples);
        if (execute) {
          const auto execution = move_group.execute(plan);
          require(execution == moveit::core::MoveItErrorCode::SUCCESS, "Execution failed");
          double rail_error = 1, arm_error = 1, drift = 0;
          for (int i = 0; i < 500; ++i) {
            require(rclcpp::wait_for_message(message, reader, "/joint_states", 1s), "Missing final state");
            auto final = *current;
            apply_joint_state(final, message);
            double bounds_excess = 0;
            for (const auto& name : final.getVariableNames()) {
              const auto& bound = final.getRobotModel()->getVariableBounds(name);
              const double value = final.getVariablePosition(name);
              if (bound.position_bounded_) bounds_excess = std::max(bounds_excess,
                std::max(bound.min_position_ - value, value - bound.max_position_));
            }
            if (bounds_excess > 0) RCLCPP_INFO(logger, "Measured bounds excess=%.12g tolerance=1e-6 (no clamping)", bounds_excess);
            rail_error = std::abs(final.getVariablePosition(names[0]) - target.getVariablePosition(names[0]));
            arm_error = 0;
            for (std::size_t j = 1; j < names.size(); ++j) arm_error = std::max(arm_error,
              std::abs(final.getVariablePosition(names[j]) - target.getVariablePosition(names[j])));
            for (const auto& name : stationary_names) drift = std::max(drift,
              std::abs(final.getVariablePosition(name) - current->getVariablePosition(name)));
            if (!inspect(scene, final, logger, "final", false, "", 1e-6).valid) {
              inspect(scene, final, logger, "invalid final");
              RCLCPP_INFO(logger, "Final values: %s", values_string(final.getVariableNames(), read_group_values(final, final.getVariableNames())).c_str());
              throw std::runtime_error("Final state invalid");
            }
            if (rail_error <= 0.0001 && arm_error <= 0.001) break;
          }
          drift = std::max(drift, observed_drift->load());
          RCLCPP_INFO(logger, "Execution=SUCCESS final_rail_error=%.9f final_arm_error=%.9f stationary_max_drift=%.9f final_cross_contacts=0", rail_error, arm_error, drift);
          require(rail_error <= 0.0001 && arm_error <= 0.001 && drift <= 0.002, "Final error or stationary drift exceeds tolerance");
        } else RCLCPP_INFO(logger, "Execution skipped (plan-only)");
      } else RCLCPP_INFO(logger, "No collision-free path returned; execution skipped; waypoint check N/A");
    }
    RCLCPP_INFO(logger, "VALIDATION PASS mode=%s", mode.c_str());
    status = 0;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "VALIDATION FAIL: %s", error.what());
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return status;
}
