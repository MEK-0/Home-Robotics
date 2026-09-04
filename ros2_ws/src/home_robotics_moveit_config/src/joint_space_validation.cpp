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

namespace
{
struct RobotConfig
{
  std::string group;
  std::vector<std::string> joints;
  std::vector<double> target;
};

RobotConfig config_for(const std::string& robot, bool invalid_test)
{
  const std::string prefix = robot + "_";
  RobotConfig config;
  config.group = robot + "_manipulator";
  config.joints = {prefix + "rail_joint", prefix + "joint1", prefix + "joint2",
                   prefix + "joint3", prefix + "joint4", prefix + "joint5",
                   prefix + "joint6", prefix + "joint7"};
  if (invalid_test) {
    // Deliberately folded Panda1 pose; this candidate is collision-checked and never executed.
    config.target = {-0.9, 0.0, 1.70, 0.0, -0.10, 0.0, 0.10, 0.0};
  } else if (robot == "panda1") {
    config.target = {-0.85, 0.10, -0.10, 0.10, -1.50, 0.05, 1.50, -0.70};
  } else {
    config.target = {0.85, -0.10, 0.10, -0.10, -1.50, -0.05, 1.50, -0.70};
  }
  return config;
}

std::string values_string(const std::vector<std::string>& names, const std::vector<double>& values)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(5);
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
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "joint_space_validation",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  std::string robot = "panda1";
  bool execute = true;
  bool invalid_test = false;
  node->get_parameter_or("robot", robot, robot);
  node->get_parameter_or("execute", execute, execute);
  node->get_parameter_or("invalid_test", invalid_test, invalid_test);
  if (robot != "panda1" && robot != "panda2") {
    RCLCPP_ERROR(node->get_logger(), "robot must be 'panda1' or 'panda2'");
    rclcpp::shutdown();
    return 2;
  }
  if (invalid_test && robot != "panda1") {
    RCLCPP_ERROR(node->get_logger(), "The invalid collision test is defined for panda1 only");
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  auto state_reader = rclcpp::Node::make_shared("joint_state_validation_reader");

  int result = 1;
  try {
    const auto config = config_for(robot, invalid_test);
    RCLCPP_INFO(node->get_logger(), "Selected planning group: %s", config.group.c_str());
    moveit::planning_interface::MoveGroupInterface move_group(node, config.group);
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(10.0);

    auto current = move_group.getCurrentState(10.0);
    if (!current) {
      throw std::runtime_error("MoveIt current state was not received");
    }
    sensor_msgs::msg::JointState authoritative_start;
    if (!rclcpp::wait_for_message(authoritative_start, state_reader, "/joint_states", 10s)) {
      throw std::runtime_error("Fresh authoritative /joint_states was not received");
    }
    apply_joint_state(*current, authoritative_start);
    const auto start_values = read_group_values(*current, config.joints);
    RCLCPP_INFO(node->get_logger(), "Current state received: %s",
                values_string(config.joints, start_values).c_str());

    auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity");
    auto current_validity = check_validity(node, validity_client, *current, config.group);
    if (!current_validity) {
      throw std::runtime_error("Current-state validity could not be checked");
    }
    RCLCPP_INFO(node->get_logger(), "Current state validity: %s",
                current_validity->valid ? "VALID" : "INVALID");
    if (!current_validity->valid) {
      throw std::runtime_error("Current state is invalid; refusing to plan");
    }

    moveit::core::RobotState target_state(*current);
    std::map<std::string, double> target_map;
    for (std::size_t index = 0; index < config.joints.size(); ++index) {
      target_map.emplace(config.joints[index], config.target[index]);
    }
    target_state.setVariablePositions(target_map);
    target_state.update();
    RCLCPP_INFO(node->get_logger(), "Target state: %s",
                values_string(config.joints, config.target).c_str());
    if (!target_state.satisfiesBounds()) {
      RCLCPP_ERROR(node->get_logger(), "Target violates robot-model joint bounds; not executing");
      result = invalid_test ? 0 : 3;
    } else {
      auto target_validity = check_validity(node, validity_client, target_state, config.group);
      if (!target_validity) {
        throw std::runtime_error("Target-state validity could not be checked");
      }
      RCLCPP_INFO(node->get_logger(), "Target state validity: %s (contacts=%zu)",
                  target_validity->valid ? "VALID" : "INVALID", target_validity->contacts.size());
      for (const auto& contact : target_validity->contacts) {
        RCLCPP_WARN(node->get_logger(), "Collision contact: %s <-> %s",
                    contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
      }
      if (!target_validity->valid) {
        RCLCPP_WARN(node->get_logger(), "Invalid target rejected before planning; execution skipped");
        result = invalid_test ? 0 : 4;
      } else if (invalid_test) {
        RCLCPP_ERROR(node->get_logger(), "Invalid-test candidate was unexpectedly valid; execution skipped");
        result = 5;
      } else {
        move_group.setStartState(*current);
        if (!move_group.setJointValueTarget(target_map)) {
          throw std::runtime_error("MoveGroup rejected the joint-value target");
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = move_group.plan(plan);
        const bool planned = plan_result == moveit::core::MoveItErrorCode::SUCCESS;
        RCLCPP_INFO(node->get_logger(),
                    "Planning result: %s, planning_time=%.6f s, trajectory_points=%zu",
                    planned ? "SUCCESS" : "FAILURE", plan.planning_time,
                    plan.trajectory.joint_trajectory.points.size());
        if (!planned) {
          result = 6;
        } else if (!execute) {
          RCLCPP_INFO(node->get_logger(), "Plan-only mode selected; execution skipped");
          result = 0;
        } else {
          const auto execution_result = move_group.execute(plan);
          const bool executed = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
          RCLCPP_INFO(node->get_logger(), "Execution result: %s",
                      executed ? "SUCCESS" : "FAILURE");
          if (!executed) {
            result = 7;
          } else {
            auto final_state = move_group.getCurrentState(5.0);
            if (!final_state) {
              throw std::runtime_error("Final /joint_states-backed state was not received");
            }
            std::vector<double> final_values;
            double max_rail_error = std::numeric_limits<double>::infinity();
            double max_arm_error = std::numeric_limits<double>::infinity();
            for (int sample = 0; sample < 50; ++sample) {
              sensor_msgs::msg::JointState authoritative_final;
              if (!rclcpp::wait_for_message(
                  authoritative_final, state_reader, "/joint_states", 200ms)) {
                continue;
              }
              apply_joint_state(*final_state, authoritative_final);
              final_values = read_group_values(*final_state, config.joints);
              max_rail_error = std::abs(final_values.front() - config.target.front());
              max_arm_error = 0.0;
              for (std::size_t index = 1; index < final_values.size(); ++index) {
                max_arm_error = std::max(max_arm_error,
                  std::abs(final_values[index] - config.target[index]));
              }
              if (max_rail_error <= 0.01 && max_arm_error <= 0.02) {
                break;
              }
            }
            if (final_values.empty()) {
              throw std::runtime_error("Fresh final authoritative /joint_states was not received");
            }
            RCLCPP_INFO(node->get_logger(), "Final authoritative state: %s",
                        values_string(config.joints, final_values).c_str());
            RCLCPP_INFO(node->get_logger(),
                        "Final error: rail=%.8f m, max_arm=%.8f rad", max_rail_error, max_arm_error);
            result = (max_rail_error <= 0.01 && max_arm_error <= 0.02) ? 0 : 8;
          }
        }
      }
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Validation failed: %s", error.what());
    result = 9;
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return result;
}
