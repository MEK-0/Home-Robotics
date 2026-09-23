#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "home_robotics_task_executor/task_types.hpp"

namespace home_robotics_task_executor {
struct ManipulationResult { bool success{false}; std::string failure_reason; std::string message; geometry_msgs::msg::Pose final_object_pose{}; };

// Boundary to Phase 4.  It owns neither planning nor physics; it invokes the
// existing Phase 4 pick/place pipeline and translates its structured report.
class ManipulationAdapter {
 public:
  virtual ~ManipulationAdapter() = default;
  virtual ManipulationResult pick_and_place(const TaskRequest & request, bool execute,
    const std::function<bool()> & cancellation_requested) const = 0;
};

class Phase4ManipulationAdapter final : public ManipulationAdapter {
 public:
  explicit Phase4ManipulationAdapter(rclcpp::Logger logger) : logger_(std::move(logger)) {}
  // Cancellation is intentionally honoured only for planning-only work. The
  // Phase 4 execution pipeline owns controller cancellation and currently has
  // no safe externally interruptible boundary while a trajectory is active.
  ManipulationResult pick_and_place(const TaskRequest & request, bool execute,
    const std::function<bool()> & cancellation_requested) const;
 private:
  rclcpp::Logger logger_;
};
}  // namespace home_robotics_task_executor
