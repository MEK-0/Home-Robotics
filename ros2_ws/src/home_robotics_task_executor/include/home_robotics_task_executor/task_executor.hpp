#pragma once

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "home_robotics_interfaces/action/pick_and_place.hpp"
#include "home_robotics_task_executor/manipulation_adapter.hpp"

namespace home_robotics_task_executor {
class TaskExecutor : public rclcpp::Node {
 public:
  using PickAndPlace = home_robotics_interfaces::action::PickAndPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PickAndPlace>;
  explicit TaskExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<ManipulationAdapter> adapter = nullptr);
 public:
  // Narrow test hook: validation is side-effect free and runs before adapter delegation.
  bool validate_for_test(const TaskRequest & request, TaskResult & result) const { return validate(request, result); }
 private:
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const PickAndPlace::Goal> goal);
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
  bool validate(const TaskRequest & request, TaskResult & result) const;
  void feedback(const std::shared_ptr<GoalHandle> & goal, const std::string & id, TaskState state, float progress, const std::string & message) const;
  std::shared_ptr<rclcpp_action::Server<PickAndPlace>> server_;
  std::unique_ptr<ManipulationAdapter> adapter_;
  bool execute_{false};
  std::atomic_bool active_{false};
  std::atomic_uint64_t next_task_id_{1};
};
}  // namespace home_robotics_task_executor
