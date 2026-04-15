#include "behaviortree_ros2/tree_execution_server.hpp"

#include <chrono>

#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<BT::TreeExecutionServer>(options);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
  return 0;
}
