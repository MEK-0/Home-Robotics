#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include "home_robotics_task_executor/task_executor.hpp"

using namespace std::chrono_literals;
namespace hrt = home_robotics_task_executor;

struct FakeControl {
  std::mutex mutex;
  std::condition_variable entered_cv;
  std::condition_variable release_cv;
  bool block{false};
  bool entered{false};
  bool released{false};
  int calls{0};
  hrt::ManipulationResult result{true, "NONE", "fake success", geometry_msgs::msg::Pose{}};
};

class FakeAdapter final : public hrt::ManipulationAdapter {
 public:
  explicit FakeAdapter(std::shared_ptr<FakeControl> control) : control_(std::move(control)) {}
  hrt::ManipulationResult pick_and_place(const hrt::TaskRequest &, bool,
    const std::function<bool()> & canceled) const override {
    std::unique_lock<std::mutex> lock(control_->mutex);
    ++control_->calls;
    control_->entered = true;
    control_->entered_cv.notify_all();
    control_->release_cv.wait(lock, [&] { return !control_->block || control_->released; });
    if (canceled()) return {false, "CANCELED", "fake cancellation", geometry_msgs::msg::Pose{}};
    return control_->result;
  }
 private:
  std::shared_ptr<FakeControl> control_;
};

class TaskExecutorActionTest : public ::testing::Test {
 protected:
  using Action = hrt::TaskExecutor::PickAndPlace;
  using Client = rclcpp_action::Client<Action>;

  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    control_ = std::make_shared<FakeControl>();
    rclcpp::NodeOptions options;
    options.append_parameter_override("execute", false);
    executor_node_ = std::make_shared<hrt::TaskExecutor>(options, std::make_unique<FakeAdapter>(control_));
    client_node_ = std::make_shared<rclcpp::Node>("task_executor_action_test_client");
    executor_->add_node(executor_node_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this] { executor_->spin(); });
    client_ = rclcpp_action::create_client<Action>(client_node_, "/home_robotics/pick_and_place");
    ASSERT_TRUE(client_->wait_for_action_server(2s));
  }

  void TearDown() override {
    executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_->remove_node(client_node_);
    executor_->remove_node(executor_node_);
    client_.reset();
    client_node_.reset();
    executor_node_.reset();
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> send(
    const std::string & object = "cube", const std::string & target = "surface_left_2",
    typename Client::SendGoalOptions options = {}) {
    Action::Goal goal;
    goal.object_id = object;
    goal.target_id = target;
    auto future = client_->async_send_goal(goal, options);
    EXPECT_EQ(future.wait_for(2s), std::future_status::ready);
    return future.get();
  }

  typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult result(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> & handle) {
    auto future = client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(2s), std::future_status::ready);
    return future.get();
  }

  void wait_until_entered() {
    std::unique_lock<std::mutex> lock(control_->mutex);
    ASSERT_TRUE(control_->entered_cv.wait_for(lock, 2s, [this] { return control_->entered; }));
  }

  void release() {
    std::lock_guard<std::mutex> lock(control_->mutex);
    control_->released = true;
    control_->release_cv.notify_all();
  }

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<FakeControl> control_;
  std::shared_ptr<hrt::TaskExecutor> executor_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<Client> client_;
};

TEST_F(TaskExecutorActionTest, SuccessPropagatesPoseTaskIdAndMonotonicFeedback) {
  control_->result.final_object_pose.position.x = 1.25;
  std::mutex feedback_mutex;
  std::vector<float> progress;
  Client::SendGoalOptions options;
  options.feedback_callback = [&](auto, const auto feedback) {
    std::lock_guard<std::mutex> lock(feedback_mutex);
    progress.push_back(feedback->progress);
  };
  auto handle = send("cube", "surface_left_2", options);
  ASSERT_TRUE(static_cast<bool>(handle));
  const auto outcome = result(handle);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(outcome.result->success);
  EXPECT_EQ(outcome.result->final_state, "SUCCEEDED");
  EXPECT_EQ(outcome.result->failure_reason, "NONE");
  EXPECT_FALSE(outcome.result->task_id.empty());
  EXPECT_DOUBLE_EQ(outcome.result->final_object_pose.position.x, 1.25);
  ASSERT_FALSE(progress.empty());
  EXPECT_GE(progress.front(), 0.0F);
  for (size_t i = 1; i < progress.size(); ++i) EXPECT_GE(progress[i], progress[i - 1]);
  EXPECT_FLOAT_EQ(progress.back(), 100.0F);
}

TEST_F(TaskExecutorActionTest, InvalidRequestsDoNotCallAdapter) {
  const std::vector<std::pair<hrt::TaskRequest, std::string>> cases = {
    {{hrt::TaskType::PICK_AND_PLACE, "does_not_exist", "surface_left_2", "panda1"}, "OBJECT_NOT_FOUND"},
    {{hrt::TaskType::PICK_AND_PLACE, "apple", "surface_left_2", "panda1"}, "UNSUPPORTED_TASK"},
    {{hrt::TaskType::PICK_AND_PLACE, "purple_ball", "surface_left_1", "panda1"}, "UNSUPPORTED_TASK"},
    {{hrt::TaskType::PICK_AND_PLACE, "cube", "does_not_exist", "panda1"}, "TARGET_NOT_FOUND"},
    {{hrt::TaskType::PICK_AND_PLACE, "", "surface_left_2", "panda1"}, "INVALID_REQUEST"},
    {{hrt::TaskType::PICK_AND_PLACE, "cube", "", "panda1"}, "INVALID_REQUEST"},
  };
  for (const auto & item : cases) {
    hrt::TaskResult task;
    EXPECT_FALSE(executor_node_->validate_for_test(item.first, task));
    EXPECT_EQ(hrt::to_string(task.failure_reason), item.second);
  }
  std::lock_guard<std::mutex> lock(control_->mutex);
  EXPECT_EQ(control_->calls, 0);
}

TEST_F(TaskExecutorActionTest, ManipulationFailureIsMappedAndPreserved) {
  control_->result = {false, "NO_CONTACT", "fake physical grasp failure", geometry_msgs::msg::Pose{}};
  const auto outcome = result(send());
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(outcome.result->failure_reason, "MANIPULATION_FAILED");
  EXPECT_EQ(outcome.result->manipulation_failure_reason, "NO_CONTACT");
}

TEST_F(TaskExecutorActionTest, SequentialTaskIdsAreUnique) {
  control_->block = true;
  auto first_handle = send();
  wait_until_entered();
  release();
  const auto first = result(first_handle);
  {
    std::lock_guard<std::mutex> lock(control_->mutex);
    control_->entered = false;
    control_->released = false;
  }
  auto second_handle = send();
  wait_until_entered();
  release();
  const auto second = result(second_handle);
  EXPECT_FALSE(first.result->task_id.empty());
  EXPECT_FALSE(second.result->task_id.empty());
  EXPECT_NE(first.result->task_id, second.result->task_id);
}

TEST_F(TaskExecutorActionTest, BusyGoalIsRejectedWithoutSecondAdapterCall) {
  control_->block = true;
  auto first = send();
  ASSERT_TRUE(static_cast<bool>(first));
  wait_until_entered();
  auto second = send();
  EXPECT_FALSE(static_cast<bool>(second));
  release();
  EXPECT_EQ(result(first).code, rclcpp_action::ResultCode::SUCCEEDED);
  std::lock_guard<std::mutex> lock(control_->mutex);
  EXPECT_EQ(control_->calls, 1);
}

TEST_F(TaskExecutorActionTest, CancellationIsProcessedWhileAdapterIsBlocked) {
  control_->block = true;
  auto handle = send();
  ASSERT_TRUE(static_cast<bool>(handle));
  wait_until_entered();
  auto cancel = client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel.wait_for(2s), std::future_status::ready);
  ASSERT_EQ(cancel.get()->goals_canceling.size(), 1u);
  release();
  const auto outcome = result(handle);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(outcome.result->final_state, "CANCELED");
  EXPECT_EQ(outcome.result->failure_reason, "CANCELED");
}
