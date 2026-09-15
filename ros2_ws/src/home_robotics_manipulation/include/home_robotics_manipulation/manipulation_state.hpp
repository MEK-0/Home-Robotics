#pragma once
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
// Declarative lifecycle only: no robot actions or automatic recovery.
constexpr bool transition_allowed(ManipulationState from, ManipulationState to) {
  using S = ManipulationState;
  if (to == S::FAILED) return from != S::DONE && from != S::FAILED;
  if (from == S::DONE || from == S::FAILED) return to == S::IDLE;
  return static_cast<int>(to) == static_cast<int>(from) + 1;
}
}
