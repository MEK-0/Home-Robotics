#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace home_robotics_mujoco_hardware
{
class MujocoTopicSystem : public hardware_interface::SystemInterface
{
public:
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  bool safe_rail_commands() const;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> indices_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;
  std::vector<double> pending_positions_;
  std::vector<double> pending_velocities_;
  mutable std::mutex state_mutex_;
  std::atomic_bool state_received_{false};
  double minimum_separation_{0.7};
};
}  // namespace home_robotics_mujoco_hardware
