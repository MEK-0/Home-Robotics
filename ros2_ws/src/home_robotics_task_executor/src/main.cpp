#include <rclcpp/rclcpp.hpp>
#include "home_robotics_task_executor/task_executor.hpp"
int main(int argc, char ** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<home_robotics_task_executor::TaskExecutor>()); rclcpp::shutdown(); return 0; }
