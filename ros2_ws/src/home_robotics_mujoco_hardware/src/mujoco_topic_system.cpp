#include "home_robotics_mujoco_hardware/mujoco_topic_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace home_robotics_mujoco_hardware
{
std::vector<hardware_interface::StateInterface> MujocoTopicSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &positions_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoTopicSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (!info_.joints[i].command_interfaces.empty()) {
      interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &commands_[i]);
    }
  }
  return interfaces;
}

hardware_interface::CallbackReturn MujocoTopicSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  minimum_separation_ = std::stod(info_.hardware_parameters.at("minimum_rail_separation"));
  const auto count = info_.joints.size();
  positions_.assign(count, 0.0);
  velocities_.assign(count, 0.0);
  commands_.assign(count, std::numeric_limits<double>::quiet_NaN());
  pending_positions_.assign(count, 0.0);
  pending_velocities_.assign(count, 0.0);
  for (std::size_t index = 0; index < count; ++index) {
    const auto & joint = info_.joints[index];
    joint_names_.push_back(joint.name);
    indices_[joint.name] = index;
    const auto & initial = joint.state_interfaces[0].initial_value;
    if (!initial.empty()) {
      positions_[index] = pending_positions_[index] = std::stod(initial);
      commands_[index] = std::stod(initial);
    }
    if (joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' must expose position and velocity state", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces.size() > 1 ||
      (!joint.command_interfaces.empty() &&
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION))
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' has an invalid command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  auto node = get_node();
  command_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
    "/mujoco/command", rclcpp::SystemDefaultsQoS());
  state_subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "/mujoco/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&MujocoTopicSystem::state_callback, this, std::placeholders::_1));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoTopicSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  commands_ = pending_positions_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MujocoTopicSystem::state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message->name.size() != message->position.size() ||
    message->name.size() != message->velocity.size()) {return;}
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t source = 0; source < message->name.size(); ++source) {
    const auto found = indices_.find(message->name[source]);
    if (found != indices_.end()) {
      pending_positions_[found->second] = message->position[source];
      pending_velocities_[found->second] = message->velocity[source];
    }
  }
  state_received_ = true;
}

hardware_interface::return_type MujocoTopicSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  positions_ = pending_positions_;
  velocities_ = pending_velocities_;
  return hardware_interface::return_type::OK;
}

bool MujocoTopicSystem::safe_rail_commands() const
{
  const double panda1 = commands_[indices_.at("panda1_rail_joint")];
  const double panda2 = commands_[indices_.at("panda2_rail_joint")];
  return std::isfinite(panda1) && std::isfinite(panda2) &&
    panda2 - panda1 >= minimum_separation_;
}

hardware_interface::return_type MujocoTopicSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!state_received_) {return hardware_interface::return_type::OK;}
  const bool rail_safe = safe_rail_commands();
  if (!rail_safe) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Holding rail commands: carriage separation would be below %.3f m", minimum_separation_);
  }
  sensor_msgs::msg::JointState message;
  message.header.stamp = get_clock()->now();
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    if (!info_.joints[index].command_interfaces.empty()) {
      message.name.push_back(joint_names_[index]);
      const bool is_rail = joint_names_[index] == "panda1_rail_joint" ||
        joint_names_[index] == "panda2_rail_joint";
      message.position.push_back(!rail_safe && is_rail ? positions_[index] : commands_[index]);
    }
  }
  command_publisher_->publish(message);
  return hardware_interface::return_type::OK;
}
}  // namespace home_robotics_mujoco_hardware

PLUGINLIB_EXPORT_CLASS(
  home_robotics_mujoco_hardware::MujocoTopicSystem,
  hardware_interface::SystemInterface)
