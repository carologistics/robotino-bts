#include "behaviortree_ros2/tree_execution_server.hpp"

#include <chrono>

#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace robotino_behavior_tree
{

class RobotinoTreeServer : public BT::TreeExecutionServer
{
public:
  using BT::TreeExecutionServer::TreeExecutionServer;

protected:
  void onTreeCreated(BT::Tree& tree) override
  {
    auto blackboard = tree.rootBlackboard();
    setIfNotEmpty(blackboard, "object_prompt", goalObjectPrompt());
    setIfNotEmpty(blackboard, "machine_input_tf", goalMachineInputTf());
    setIfNotEmpty(blackboard, "reference_frame", goalReferenceFrame());
    setIfNotEmpty(blackboard, "object_type", goalObjectType());
    setIfNotEmpty(blackboard, "lego_color", goalLegoColor());
    setIfNotEmpty(blackboard, "action", goalAction());
  }

private:
  static void setIfNotEmpty(
    const BT::Blackboard::Ptr & blackboard,
    const std::string & key,
    const std::string & value)
  {
    if(!value.empty()) {
      blackboard->set(key, value);
    }
  }
};

}  // namespace robotino_behavior_tree

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<robotino_behavior_tree::RobotinoTreeServer>(options);

  auto node = action_server->node();
  std::string ns = node->get_namespace();
  // Strip leading slash if present
  if (!ns.empty() && ns[0] == '/') {
    ns = ns.substr(1);
  }
  action_server->globalBlackboard()->set("namespace", ns);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
  return 0;
}
