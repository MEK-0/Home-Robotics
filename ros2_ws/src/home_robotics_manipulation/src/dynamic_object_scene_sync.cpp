#include "home_robotics_manipulation/object_registry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <std_srvs/srv/set_bool.hpp>
using namespace std::chrono_literals;
namespace hm = home_robotics_manipulation;
class DynamicSceneSync : public rclcpp::Node {
public:
  DynamicSceneSync() : Node("dynamic_object_scene_sync"), registry_(declare_parameter("objects_config",
    ament_index_cpp::get_package_share_directory("home_robotics_control") + "/config/objects.yaml")) {
    const auto hz = declare_parameter("sync_rate", 20.0);
    if (hz < 10 || hz > 30) throw std::runtime_error("sync_rate must be 10..30 Hz");
    for (const auto& [id, metadata] : registry_.objects()) {
      subscriptions_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mujoco/objects/" + id + "/pose", 1,
        [this, id](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {
          try { hm::world_object(registry_.at(id), *pose); poses_[id] = *pose; }
          catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "INVALID_OBJECT_STATE %s: %s", id.c_str(), e.what()); }
        }));
    }
    get_ = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    pause_service_ = create_service<std_srvs::srv::SetBool>("/dynamic_object_scene_sync/pause",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        paused_ = request->data;
        response->success = !busy_;
        response->message = busy_ ? "Draining outstanding scene update; retry" : "Sync pause acknowledged";
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Registry=%zu objects; %.1f Hz world sync; MuJoCo owns poses", registry_.objects().size(), hz);
  }
private:
  void tick() {
    if (busy_) {
      if (std::chrono::steady_clock::now() - sent_ > 3s) {
        RCLCPP_ERROR(get_logger(), "SCENE_SYNC_FAILED service response timeout");
        // Invalidate late callbacks and release timed-out requests.
        ++generation_; get_->prune_pending_requests(); apply_->prune_pending_requests(); busy_ = false;
      }
      return;
    }
    if (paused_) return;
    if (!get_->service_is_ready() || !apply_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "SCENE_SYNC_FAILED waiting for MoveIt services"); return;
    }
    if (poses_.size() != registry_.objects().size()) return;
    for (const auto& [id, pose] : poses_) {
      const double age = (now() - rclcpp::Time(pose.header.stamp)).seconds();
      if (age < -0.1 || age > 0.5) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "SCENE_SYNC_FAILED stale object pose: %s", id.c_str()); return;
      }
    }
    busy_ = true; sent_ = std::chrono::steady_clock::now();
    const auto generation = ++generation_;
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    get_->async_send_request(request, [this, generation](rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future) {
      if (generation != generation_) return;
      std::set<std::string> attached;
      for (const auto& item : future.get()->scene.robot_state.attached_collision_objects) attached.insert(item.object.id);
      auto update = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
      // Future manipulation owns attached objects; never resurrect them in world.
      update->scene = hm::world_update(registry_, poses_, attached);
      apply_->async_send_request(update, [this, generation](rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedFuture result) {
        if (generation != generation_) return;
        busy_ = false;
        if (!result.get()->success) RCLCPP_ERROR(get_logger(), "SCENE_SYNC_FAILED ApplyPlanningScene rejected diff");
        else if (!announced_) { RCLCPP_INFO(get_logger(), "Dynamic world diff accepted; stable IDs, static scene untouched"); announced_ = true; }
      });
    });
  }
  hm::ObjectRegistry registry_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> poses_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subscriptions_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  bool paused_ = false;
  bool busy_ = false, announced_ = false;
  std::size_t generation_ = 0;
  std::chrono::steady_clock::time_point sent_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<DynamicSceneSync>()); }
  catch (const std::exception& e) { RCLCPP_FATAL(rclcpp::get_logger("dynamic_scene_sync"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
