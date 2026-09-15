// Offline negative collision test only. No planning, gripper command or execution.
#include <chrono>
#include <thread>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
using namespace std::chrono_literals;
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cube_collision_validation", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });
  int result = 1;
  try {
    moveit::planning_interface::MoveGroupInterface group(node, "panda1_manipulator");
    auto state = group.getCurrentState(10);
    if (!state) throw std::runtime_error("Missing current state");
    auto reader = rclcpp::Node::make_shared("cube_validation_state_reader");
    sensor_msgs::msg::JointState joints;
    if (!rclcpp::wait_for_message(joints, reader, "/joint_states", 10s))
      throw std::runtime_error("Missing authoritative full joint state");
    std::map<std::string, double> positions;
    for (std::size_t i = 0; i < joints.name.size(); ++i) positions.emplace(joints.name[i], joints.position.at(i));
    for (const auto& name : state->getVariableNames())
      if (!positions.count(name)) throw std::runtime_error("Incomplete authoritative joint state");
    state->setVariablePositions(positions); state->update();
    auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!client->wait_for_service(10s)) throw std::runtime_error("Missing scene service");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 1023;
    auto future = client->async_send_request(request);
    if (future.wait_for(10s) != std::future_status::ready) throw std::runtime_error("Scene timeout");
    planning_scene::PlanningScene scene(group.getRobotModel()); scene.setPlanningSceneMsg(future.get()->scene);
    collision_detection::CollisionRequest initial_query;
    collision_detection::CollisionResult initial_result;
    scene.checkCollision(initial_query, initial_result, *state);
    if (initial_result.collision) throw std::runtime_error("Initial full robot state in collision");
    const auto cube = scene.getWorld()->getObject("cube");
    if (!cube) throw std::runtime_error("Cube missing");
    const Eigen::Vector3d center = cube->global_shape_poses_[0].translation();
    const auto* jmg = state->getJointModelGroup("panda1_manipulator");
    bool found = false;
    for (int attempt = 0; attempt < 80 && !found; ++attempt) {
      auto candidate = *state;
      // Fixed series of hand orientations and rail seeds for negative IK fixtures.
      candidate.setVariablePosition("panda1_rail_joint", -1.4 + 0.1 * (attempt % 15));
      Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
      target.translation() = center;
      target.linear() = (Eigen::AngleAxisd((attempt % 8) * 0.785398163397, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX())).toRotationMatrix();
      if (!candidate.setFromIK(jmg, target, "panda1_hand", 0.08)) continue;
      candidate.update();
      if (!candidate.satisfiesBounds(1e-6)) continue;
      collision_detection::CollisionRequest query;
      query.contacts = true; query.max_contacts = 1000; query.max_contacts_per_pair = 1;
      collision_detection::CollisionResult collision;
      scene.checkCollision(query, collision, candidate);
      for (const auto& [pair, contacts] : collision.contacts) {
        const auto robot = pair.first == "cube" ? pair.second : pair.second == "cube" ? pair.first : "";
        if (robot.rfind("panda1_", 0) == 0) found = true;
      }
      if (!found) continue;
      for (const auto& [pair, contacts] : collision.contacts)
        RCLCPP_INFO(node->get_logger(), "INVALID contact: %s <-> %s", pair.first.c_str(), pair.second.c_str());
      for (const auto& name : jmg->getVariableNames())
        RCLCPP_INFO(node->get_logger(), "offline state %s=%.12f", name.c_str(), candidate.getVariablePosition(name));
      auto validity = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
      if (!validity->wait_for_service(10s)) throw std::runtime_error("Missing validity service");
      auto check = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      moveit::core::robotStateToRobotStateMsg(candidate, check->robot_state);
      auto response = validity->async_send_request(check);
      if (response.wait_for(10s) != std::future_status::ready || response.get()->valid)
        throw std::runtime_error("Server did not reject cube intersection");
      RCLCPP_INFO(node->get_logger(), "PASS cube collision rejected locally and by service; never executed");
      result = 0;
    }
    if (!found) throw std::runtime_error("Could not construct in-bounds cube intersection");
  } catch (const std::exception& e) { RCLCPP_ERROR(node->get_logger(), "FAIL: %s", e.what()); }
  executor.cancel(); spinner.join(); rclcpp::shutdown(); return result;
}
