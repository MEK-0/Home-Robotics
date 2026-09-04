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

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
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
  std::string tcp;
  std::vector<std::string> joints;
  double default_x;
};

RobotConfig config_for(const std::string& robot)
{
  const std::string prefix = robot + "_";
  return {
    robot + "_manipulator",
    robot + "_tcp",
    {prefix + "rail_joint", prefix + "joint1", prefix + "joint2", prefix + "joint3",
     prefix + "joint4", prefix + "joint5", prefix + "joint6", prefix + "joint7"},
    robot == "panda1" ? 0.04 : -0.04
  };
}

void apply_joint_state(moveit::core::RobotState& state, const sensor_msgs::msg::JointState& message)
{
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < message.name.size(); ++index) {
    positions.emplace(message.name[index], message.position[index]);
  }
  state.setVariablePositions(positions);
  state.update();
}

geometry_msgs::msg::Pose pose_from_transform(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond quaternion(transform.rotation());
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

std::string pose_string(const geometry_msgs::msg::Pose& pose)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(8)
         << "position=[" << pose.position.x << ", " << pose.position.y << ", "
         << pose.position.z << "], quaternion=[" << pose.orientation.x << ", "
         << pose.orientation.y << ", " << pose.orientation.z << ", "
         << pose.orientation.w << ']';
  return stream.str();
}

std::string joint_string(
  const moveit::core::RobotState& state, const std::vector<std::string>& joints)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(8);
  for (std::size_t index = 0; index < joints.size(); ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << joints[index] << '=' << state.getVariablePosition(joints[index]);
  }
  return stream.str();
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
    return nullptr;
  }
  auto future = client->async_send_request(request);
  if (future.wait_for(10s) != std::future_status::ready) {
    return nullptr;
  }
  return future.get();
}

void pose_error(
  const geometry_msgs::msg::Pose& target, const geometry_msgs::msg::Pose& actual,
  double& error_x, double& error_y, double& error_z,
  double& position_error, double& orientation_error)
{
  error_x = actual.position.x - target.position.x;
  error_y = actual.position.y - target.position.y;
  error_z = actual.position.z - target.position.z;
  position_error = std::sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
  Eigen::Quaterniond target_q(
    target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
  Eigen::Quaterniond actual_q(
    actual.orientation.w, actual.orientation.x, actual.orientation.y, actual.orientation.z);
  target_q.normalize();
  actual_q.normalize();
  const double dot = std::clamp(std::abs(target_q.dot(actual_q)), 0.0, 1.0);
  orientation_error = 2.0 * std::acos(dot);
}
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "cartesian_pose_validation",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  std::string robot = "panda1";
  bool execute = false;
  bool invalid_test = false;
  bool expect_invalid = false;
  double offset_x = std::numeric_limits<double>::quiet_NaN();
  double offset_y = 0.0;
  double offset_z = 0.02;
  node->get_parameter_or("robot", robot, robot);
  node->get_parameter_or("execute", execute, execute);
  node->get_parameter_or("invalid_test", invalid_test, invalid_test);
  node->get_parameter_or("expect_invalid", expect_invalid, expect_invalid);
  node->get_parameter_or("offset_x", offset_x, offset_x);
  node->get_parameter_or("offset_y", offset_y, offset_y);
  node->get_parameter_or("offset_z", offset_z, offset_z);
  if (robot != "panda1" && robot != "panda2") {
    RCLCPP_ERROR(node->get_logger(), "robot must be 'panda1' or 'panda2'");
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  auto state_reader = rclcpp::Node::make_shared("cartesian_joint_state_reader");
  int result = 1;

  try {
    const auto config = config_for(robot);
    if (std::isnan(offset_x)) {
      offset_x = config.default_x;
    }
    if (invalid_test) {
      offset_x = robot == "panda1" ? 3.0 : -3.0;
      offset_y = 0.0;
      offset_z = 0.0;
      execute = false;
    }
    const bool expect_rejection = invalid_test || expect_invalid;
    if (expect_rejection) {
      execute = false;
    }
    RCLCPP_INFO(node->get_logger(), "Selected robot=%s group=%s tcp=%s",
                robot.c_str(), config.group.c_str(), config.tcp.c_str());

    moveit::planning_interface::MoveGroupInterface move_group(node, config.group);
    move_group.setEndEffectorLink(config.tcp);
    move_group.setPoseReferenceFrame("world");
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.02);

    auto current = move_group.getCurrentState(10.0);
    if (!current) {
      throw std::runtime_error("MoveIt current state was not received");
    }
    sensor_msgs::msg::JointState authoritative_start;
    if (!rclcpp::wait_for_message(authoritative_start, state_reader, "/joint_states", 10s)) {
      throw std::runtime_error("Fresh authoritative /joint_states was not received");
    }
    apply_joint_state(*current, authoritative_start);

    auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity");
    auto current_validity = check_validity(node, validity_client, *current, config.group);
    if (!current_validity) {
      throw std::runtime_error("Current-state validity could not be checked");
    }
    RCLCPP_INFO(node->get_logger(), "Current state validity: %s (contacts=%zu)",
                current_validity->valid ? "VALID" : "INVALID", current_validity->contacts.size());
    if (!current_validity->valid) {
      throw std::runtime_error("Current state is invalid; refusing to plan");
    }

    const auto current_pose = pose_from_transform(current->getGlobalLinkTransform(config.tcp));
    auto target_pose = current_pose;
    target_pose.position.x += offset_x;
    target_pose.position.y += offset_y;
    target_pose.position.z += offset_z;
    RCLCPP_INFO(node->get_logger(), "Current TCP pose: %s", pose_string(current_pose).c_str());
    RCLCPP_INFO(node->get_logger(), "Requested target pose: %s", pose_string(target_pose).c_str());

    const auto* joint_model_group = current->getRobotModel()->getJointModelGroup(config.group);
    if (!joint_model_group) {
      throw std::runtime_error("Planning group is absent from RobotModel");
    }
    moveit::core::RobotState ik_state(*current);
    const bool ik_success = ik_state.setFromIK(joint_model_group, target_pose, config.tcp, 1.0);
    RCLCPP_INFO(node->get_logger(), "IK result: %s", ik_success ? "SUCCESS" : "FAILURE");
    if (!ik_success) {
      RCLCPP_WARN(node->get_logger(), "Target rejected at IK stage; execution skipped");
      result = expect_rejection ? 0 : 3;
    } else {
      ik_state.update();
      RCLCPP_INFO(node->get_logger(), "IK 8-DOF solution: %s",
                  joint_string(ik_state, config.joints).c_str());
      if (!ik_state.satisfiesBounds(joint_model_group)) {
        RCLCPP_WARN(node->get_logger(), "IK solution violates joint bounds; execution skipped");
        result = expect_rejection ? 0 : 4;
      } else {
        auto validity = check_validity(node, validity_client, ik_state, config.group);
        if (!validity) {
          throw std::runtime_error("Target-state validity could not be checked");
        }
        RCLCPP_INFO(node->get_logger(), "IK target-state validity: %s (contacts=%zu)",
                    validity->valid ? "VALID" : "INVALID", validity->contacts.size());
        for (const auto& contact : validity->contacts) {
          RCLCPP_WARN(node->get_logger(), "Collision contact: %s <-> %s",
                      contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
        }
        if (!validity->valid) {
          RCLCPP_WARN(node->get_logger(), "Target rejected at validity stage; execution skipped");
          result = expect_rejection ? 0 : 5;
        } else {
          move_group.setStartState(*current);
          if (!move_group.setPoseTarget(target_pose, config.tcp)) {
            throw std::runtime_error("MoveGroup rejected the pose target");
          }
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          const auto plan_result = move_group.plan(plan);
          move_group.clearPoseTargets();
          const bool planned = plan_result == moveit::core::MoveItErrorCode::SUCCESS;
          RCLCPP_INFO(node->get_logger(),
                      "Planning result: %s, planning_time=%.6f s, trajectory_points=%zu",
                      planned ? "SUCCESS" : "FAILURE", plan.planning_time,
                      plan.trajectory.joint_trajectory.points.size());
          if (planned && !plan.trajectory.joint_trajectory.points.empty()) {
            const auto& names = plan.trajectory.joint_trajectory.joint_names;
            const auto& positions = plan.trajectory.joint_trajectory.points.back().positions;
            std::ostringstream planned_target;
            planned_target << std::fixed << std::setprecision(8);
            for (std::size_t index = 0; index < names.size(); ++index) {
              if (index != 0) {
                planned_target << ", ";
              }
              planned_target << names[index] << '=' << positions[index];
            }
            RCLCPP_INFO(node->get_logger(), "Final planned joint target: %s",
                        planned_target.str().c_str());
          }
          if (expect_rejection) {
            if (planned) {
              RCLCPP_ERROR(node->get_logger(),
                           "Invalid-test target unexpectedly planned; execution still skipped");
              result = 6;
            } else {
              RCLCPP_INFO(node->get_logger(), "Invalid target rejected at planning stage");
              result = 0;
            }
          } else if (!planned) {
            result = 7;
          } else if (!execute) {
            RCLCPP_INFO(node->get_logger(), "Plan-only mode selected; execution skipped");
            result = 0;
          } else {
            const auto execution_result = move_group.execute(plan);
            const bool executed = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
            RCLCPP_INFO(node->get_logger(), "Execution result: %s",
                        executed ? "SUCCESS" : "FAILURE");
            if (!executed) {
              result = 8;
            } else {
              auto final_state = move_group.getCurrentState(5.0);
              if (!final_state) {
                throw std::runtime_error("Final MoveIt state was not received");
              }
              geometry_msgs::msg::Pose actual_pose;
              double error_x = 0.0;
              double error_y = 0.0;
              double error_z = 0.0;
              double position_error = std::numeric_limits<double>::infinity();
              double orientation_error = std::numeric_limits<double>::infinity();
              bool received = false;
              for (int sample = 0; sample < 50; ++sample) {
                sensor_msgs::msg::JointState authoritative_final;
                if (!rclcpp::wait_for_message(
                    authoritative_final, state_reader, "/joint_states", 200ms)) {
                  continue;
                }
                received = true;
                apply_joint_state(*final_state, authoritative_final);
                actual_pose = pose_from_transform(final_state->getGlobalLinkTransform(config.tcp));
                pose_error(target_pose, actual_pose, error_x, error_y, error_z,
                           position_error, orientation_error);
                if (position_error <= 0.01 && orientation_error <= 0.03) {
                  break;
                }
              }
              if (!received) {
                throw std::runtime_error("Fresh final authoritative /joint_states was not received");
              }
              RCLCPP_INFO(node->get_logger(), "Actual final TCP pose: %s",
                          pose_string(actual_pose).c_str());
              RCLCPP_INFO(node->get_logger(),
                          "Cartesian error: dx=%.8f dy=%.8f dz=%.8f position=%.8f m orientation=%.8f rad",
                          error_x, error_y, error_z, position_error, orientation_error);
              result = (position_error <= 0.01 && orientation_error <= 0.03) ? 0 : 9;
            }
          }
        }
      }
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Cartesian validation failed: %s", error.what());
    result = 10;
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return result;
}
