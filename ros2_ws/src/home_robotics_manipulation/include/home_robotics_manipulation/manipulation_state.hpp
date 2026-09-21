#pragma once
#include <string>
namespace home_robotics_manipulation {
enum class ManipulationState {
  IDLE, OBJECT_SELECTED, PREGRASP_PLANNED, APPROACHING, GRIPPER_CLOSING,
  GRASP_VERIFYING, GRASP_VERIFIED, OBJECT_ATTACHED, LIFTING, TRANSPORTING,
  PREPLACE_PLANNED, PLACING, RELEASING, PLACE_VERIFYING, PLACE_VERIFIED,
  DONE, FAILED
};
enum class ManipulationFailureReason {
  NONE, OBJECT_NOT_FOUND, INVALID_OBJECT_STATE, IK_FAILED, PLANNING_FAILED,
  EXECUTION_FAILED, APPROACH_COLLISION, GRIPPER_FAILED, NO_CONTACT,
  GRASP_UNSTABLE, OBJECT_DROPPED, PLACE_FAILED, SCENE_SYNC_FAILED
};
inline std::string failure_code(const std::string& message, ManipulationState stage) {
  for(const auto* code:{"OBJECT_NOT_FOUND","INVALID_OBJECT_STATE","SCENE_SYNC_FAILED","OBJECT_DROPPED",
      "GRIPPER_FAILED","NO_CONTACT","GRASP_UNSTABLE","PLACE_FAILED"})
    if(message.find(code)!=std::string::npos)return code;
  if(stage==ManipulationState::PLACING || stage==ManipulationState::PREPLACE_PLANNED)
    return "PLACE_FAILED";
  for(const auto* code:{"IK_FAILED","PLANNING_FAILED","EXECUTION_FAILED","APPROACH_COLLISION"})
    if(message.find(code)!=std::string::npos)return code;
  return "INVALID_OBJECT_STATE";
}
// Declarative lifecycle only: no robot actions or automatic recovery.
constexpr bool transition_allowed(ManipulationState from, ManipulationState to) {
  using S = ManipulationState;
  if (to == S::FAILED) return from != S::DONE && from != S::FAILED;
  if (from == S::DONE || from == S::FAILED) return to == S::IDLE;
  return static_cast<int>(to) == static_cast<int>(from) + 1;
}
}
