#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <string>

namespace home_robotics_task_executor {

enum class TaskType { PICK, PLACE, PICK_AND_PLACE, MOVE_TO_NAMED_POSE };
enum class TaskState {
  IDLE, VALIDATING_REQUEST, RESOLVING_OBJECT, RESOLVING_TARGET, PREPARING,
  EXECUTING_PICK, EXECUTING_LIFT, EXECUTING_TRANSPORT, EXECUTING_PLACE,
  VERIFYING_RESULT, SUCCEEDED, FAILED, CANCELING, CANCELED
};
enum class TaskFailureReason {
  NONE, INVALID_REQUEST, UNSUPPORTED_TASK, OBJECT_NOT_FOUND, TARGET_NOT_FOUND,
  OBJECT_NOT_MOVABLE, INVALID_TARGET, ROBOT_UNAVAILABLE, SCENE_NOT_READY,
  CONTROLLER_NOT_READY, MANIPULATION_FAILED, TASK_BUSY, CANCELED, INTERNAL_ERROR
};

struct TaskRequest { TaskType type{TaskType::PICK_AND_PLACE}; std::string object_id; std::string target_id; std::string robot_id{"panda1"}; };
struct TaskResult {
  bool success{false}; std::string task_id; TaskState final_state{TaskState::FAILED};
  TaskFailureReason failure_reason{TaskFailureReason::INTERNAL_ERROR};
  std::string manipulation_failure_reason; std::string message;
  geometry_msgs::msg::Pose final_object_pose{};
};
struct TaskFeedback { std::string task_id; TaskState current_state{TaskState::IDLE}; float progress{0.0F}; std::string message; };

const char * to_string(TaskState state);
const char * to_string(TaskFailureReason reason);
}  // namespace home_robotics_task_executor
