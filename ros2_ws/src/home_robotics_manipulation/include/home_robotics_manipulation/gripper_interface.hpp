#pragma once
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>
namespace home_robotics_manipulation {
class GripperInterface {
 public:
  using Action = control_msgs::action::GripperCommand;
  static constexpr const char* endpoint = "/panda1_gripper_controller/gripper_cmd";
  explicit GripperInterface(rclcpp::Node::SharedPtr node)
    : client_(rclcpp_action::create_client<Action>(node, endpoint)) {}
  static double fingerPosition(double width) {
    if (!std::isfinite(width) || width < 0 || width > 0.08)
      throw std::runtime_error("GRIPPER_FAILED invalid width");
    return width / 2;
  }
  void command(double width, double effort=40) {
    using namespace std::chrono_literals;
    if (!client_->wait_for_action_server(5s)) throw std::runtime_error("GRIPPER_FAILED missing action");
    Action::Goal goal; goal.command.position=fingerPosition(width); goal.command.max_effort=effort;
    auto future=client_->async_send_goal(goal);
    if(future.wait_for(5s)!=std::future_status::ready || !(handle_=future.get()))
      throw std::runtime_error("GRIPPER_FAILED goal rejected");
  }
  void open(double width) { command(width); waitForResult(); }
  void close(double width=0) { command(width); waitForResult(); }
  void waitForResult() {
    using namespace std::chrono_literals;
    auto future=client_->async_get_result(handle_);
    if(future.wait_for(20s)!=std::future_status::ready) {
      client_->async_cancel_goal(handle_); throw std::runtime_error("GRIPPER_FAILED timeout");
    }
    auto result=future.get();
    if(result.code!=rclcpp_action::ResultCode::SUCCEEDED ||
       (!result.result->reached_goal && !result.result->stalled))
      throw std::runtime_error("GRIPPER_FAILED action result");
  }
 private:
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle_;
};
inline bool validLiftHeight(double value) { return std::isfinite(value) && value>=0.08 && value<=0.15; }
}
