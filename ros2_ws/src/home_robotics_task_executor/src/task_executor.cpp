#include "home_robotics_task_executor/task_executor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdio>
#include <thread>
#include "home_robotics_manipulation/object_registry.hpp"
#include "home_robotics_manipulation/placement_target.hpp"

namespace home_robotics_task_executor {
const char * to_string(TaskState s) { static const char * n[] = {"IDLE","VALIDATING_REQUEST","RESOLVING_OBJECT","RESOLVING_TARGET","PREPARING","EXECUTING_PICK","EXECUTING_LIFT","EXECUTING_TRANSPORT","EXECUTING_PLACE","VERIFYING_RESULT","SUCCEEDED","FAILED","CANCELING","CANCELED"}; return n[static_cast<int>(s)]; }
const char * to_string(TaskFailureReason r) { static const char * n[] = {"NONE","INVALID_REQUEST","UNSUPPORTED_TASK","OBJECT_NOT_FOUND","TARGET_NOT_FOUND","OBJECT_NOT_MOVABLE","INVALID_TARGET","ROBOT_UNAVAILABLE","SCENE_NOT_READY","CONTROLLER_NOT_READY","MANIPULATION_FAILED","TASK_BUSY","CANCELED","INTERNAL_ERROR"}; return n[static_cast<int>(r)]; }

TaskExecutor::TaskExecutor(const rclcpp::NodeOptions & options, std::unique_ptr<ManipulationAdapter> adapter) : Node("task_executor", options) {
  execute_ = declare_parameter<bool>("execute", false);
  adapter_ = adapter ? std::move(adapter) : std::make_unique<Phase4ManipulationAdapter>(get_logger());
  server_ = rclcpp_action::create_server<PickAndPlace>(this, "/home_robotics/pick_and_place",
    std::bind(&TaskExecutor::on_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskExecutor::on_cancel, this, std::placeholders::_1),
    std::bind(&TaskExecutor::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse TaskExecutor::on_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const PickAndPlace::Goal> goal) {
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse TaskExecutor::on_cancel(const std::shared_ptr<GoalHandle>) {
  return execute_ ? rclcpp_action::CancelResponse::REJECT : rclcpp_action::CancelResponse::ACCEPT;
}
void TaskExecutor::feedback(const std::shared_ptr<GoalHandle> & goal, const std::string & id, TaskState state, float progress, const std::string & message) const {
  auto value = std::make_shared<PickAndPlace::Feedback>(); value->task_id = id; value->current_state = to_string(state); value->progress = progress; value->message = message; goal->publish_feedback(value);
}
bool TaskExecutor::validate(const TaskRequest & request, TaskResult & result) const {
  if (request.type != TaskType::PICK_AND_PLACE || request.object_id.empty() || request.target_id.empty()) { result.failure_reason = TaskFailureReason::INVALID_REQUEST; result.message = "object_id and target_id are required"; return false; }
  if (request.robot_id != "panda1") { result.failure_reason = TaskFailureReason::ROBOT_UNAVAILABLE; result.message = "only panda1 is available in the Phase 5 baseline"; return false; }
  try {
    const auto config = ament_index_cpp::get_package_share_directory("home_robotics_control") + "/config/";
    home_robotics_manipulation::ObjectRegistry registry(config + "objects.yaml");
    const auto & object = registry.at(request.object_id);
    if (!object.movable) { result.failure_reason = TaskFailureReason::OBJECT_NOT_MOVABLE; result.message = "object is not movable"; return false; }
    const auto scene = YAML::LoadFile(config + "scene.yaml");
    if (!scene["scene"]["surfaces"][request.target_id]) { result.failure_reason = TaskFailureReason::TARGET_NOT_FOUND; result.message = "target_id is absent from the authoritative scene"; return false; }
    const auto source = YAML::LoadFile(config + "objects.yaml")["objects"][request.object_id]["initial"]["support_surface"].as<std::string>();
    if (request.target_id == source) { result.failure_reason = TaskFailureReason::INVALID_TARGET; result.message = "target must differ from the source support surface"; return false; }
    try { (void)home_robotics_manipulation::PlacementTarget::lookup(config + "scene.yaml", request.target_id); }
    catch (const std::exception & error) { result.failure_reason = TaskFailureReason::INVALID_TARGET; result.message = error.what(); return false; }
  } catch (const std::out_of_range &) { result.failure_reason = TaskFailureReason::OBJECT_NOT_FOUND; result.message = "object_id is absent from the authoritative registry"; return false; }
  catch (const std::exception & error) { result.failure_reason = TaskFailureReason::SCENE_NOT_READY; result.message = error.what(); return false; }
  return true;
}
void TaskExecutor::handle_accepted(const std::shared_ptr<GoalHandle> goal) {
  std::thread{[this, goal] { execute(goal); }}.detach();
}

void TaskExecutor::execute(const std::shared_ptr<GoalHandle> goal) {
  struct Release { std::atomic_bool & active; ~Release() { active.store(false); } } release{active_};
  const std::string id = "task_" + [&] { char text[7]; std::snprintf(text, sizeof(text), "%06llu", static_cast<unsigned long long>(next_task_id_.fetch_add(1))); return std::string(text); }();
  TaskRequest request; request.object_id = goal->get_goal()->object_id; request.target_id = goal->get_goal()->target_id;
  TaskResult task; task.task_id = id;
  const auto started = std::chrono::steady_clock::now();
  feedback(goal, id, TaskState::VALIDATING_REQUEST, 5.0F, "Validating task request");
  if (goal->is_canceling()) {
    auto result = std::make_shared<PickAndPlace::Result>(); result->success=false; result->task_id=id; result->final_state="CANCELED"; result->failure_reason="CANCELED"; result->message="Canceled before validation"; goal->canceled(result); return;
  }
  if (!validate(request, task)) {
    task.final_state = TaskState::FAILED; auto result = std::make_shared<PickAndPlace::Result>(); result->success=false; result->task_id=id; result->final_state=to_string(task.final_state); result->failure_reason=to_string(task.failure_reason); result->message=task.message; goal->abort(result); return;
  }
  feedback(goal, id, TaskState::RESOLVING_OBJECT, 10.0F, "Resolved object from Phase 4 registry");
  feedback(goal, id, TaskState::RESOLVING_TARGET, 15.0F, "Resolved placement target from Phase 4 scene configuration");
  feedback(goal, id, TaskState::PREPARING, 20.0F, "Handing task to Phase 4 manipulation pipeline");
  feedback(goal, id, TaskState::EXECUTING_PICK, 35.0F, "Phase 4 pick and grasp verification in progress");
  if (goal->is_canceling()) {
    auto result = std::make_shared<PickAndPlace::Result>(); result->success=false; result->task_id=id; result->final_state="CANCELED"; result->failure_reason="CANCELED"; result->message="Canceled at safe boundary before Phase 4 execution"; goal->canceled(result); return;
  }
  const auto manipulation = adapter_->pick_and_place(request, execute_, [goal] { return goal->is_canceling(); });
  auto result = std::make_shared<PickAndPlace::Result>(); result->task_id=id; result->success=manipulation.success; result->final_object_pose=manipulation.final_object_pose; result->manipulation_failure_reason=manipulation.failure_reason; result->message=manipulation.message;
  if (manipulation.failure_reason == "CANCELED") {
    result->final_state="CANCELED"; result->failure_reason="CANCELED"; goal->canceled(result);
  } else if (manipulation.success) { feedback(goal,id,TaskState::VERIFYING_RESULT,95.0F,"Phase 4 final placement verification complete"); task.final_state=TaskState::SUCCEEDED; result->final_state=to_string(task.final_state); result->failure_reason="NONE"; feedback(goal,id,task.final_state,100.0F,"Task completed"); goal->succeed(result); }
  else { task.final_state=TaskState::FAILED; task.failure_reason=TaskFailureReason::MANIPULATION_FAILED; result->final_state=to_string(task.final_state); result->failure_reason=to_string(task.failure_reason); goal->abort(result); }
  const auto duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  RCLCPP_INFO(get_logger(), "task_id=%s object_id=%s target_id=%s success=%d final_state=%s failure_reason=%s manipulation_failure_reason=%s duration=%.3f", id.c_str(), request.object_id.c_str(), request.target_id.c_str(), result->success, result->final_state.c_str(), result->failure_reason.c_str(), result->manipulation_failure_reason.c_str(), duration);
}
}  // namespace home_robotics_task_executor
