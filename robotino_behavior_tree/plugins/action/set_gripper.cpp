#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "gigatino_msgs/action/gripper.hpp"
#include "gigatino_msgs/msg/status_code.hpp"

namespace robotino_behavior_tree
{

inline BT::RosNodeParams withDefaultActionName(
  const BT::RosNodeParams & params,
  const std::string & action_name)
{
  auto updated = params;
  if(updated.default_port_value.empty()) {
    updated.default_port_value = action_name;
  }
  return updated;
}

class SetGripper : public BT::RosActionNode<gigatino_msgs::action::Gripper>
{
public:
  using Action = gigatino_msgs::action::Gripper;
  using Base = BT::RosActionNode<Action>;
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using WrappedResult = typename Base::WrappedResult;
  using Goal = typename Base::Goal;

  SetGripper(const std::string & name, const NodeConfig & config, const RosNodeParams & params)
  : Base(name, config, withDefaultActionName(params, "gigatino/gripper"))
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<bool>("open", false, "True to open, false to close the gripper"),
        BT::OutputPort<unsigned>("status_code", "gigatino status code"),
        BT::OutputPort<std::string>("message", "Result message from the action server"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.open = getInput<bool>("open").value_or(false);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    const auto status_code =
      result.result ? result.result->status_code : gigatino_msgs::msg::StatusCode::UNKNOWN;
    const auto message = result.result ? result.result->message : std::string{};

    setOutput("status_code", static_cast<unsigned>(status_code));
    setOutput("message", message);

    if(status_code != gigatino_msgs::msg::StatusCode::SUCCESS) {
      RCLCPP_ERROR(logger(), "%s completed with status_code=%u: %s", name().c_str(),
                   static_cast<unsigned>(status_code), message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(
    BT::ActionNodeErrorCode error,
    const std::optional<WrappedResult> & result) override
  {
    if(result && result->result) {
      setOutput("status_code", static_cast<unsigned>(result->result->status_code));
      setOutput("message", result->result->message);
      RCLCPP_ERROR(logger(), "%s failed: %s (%s)", name().c_str(), BT::toStr(error),
                   result->result->message.c_str());
    } else {
      setOutput("status_code", static_cast<unsigned>(gigatino_msgs::msg::StatusCode::UNKNOWN));
      setOutput("message", std::string{});
      RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), BT::toStr(error));
    }
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::SetGripper, "SetGripper");
