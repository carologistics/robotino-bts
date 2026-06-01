#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "gigatino_msgs/action/move.hpp"
#include "gigatino_msgs/msg/status_code.hpp"

namespace robotino_behavior_tree
{

inline BT::RosNodeParams withDefaultActionName(const BT::RosNodeParams& params,
                                               const std::string& action_name)
{
  auto updated = params;
  if(updated.default_port_value.empty())
  {
    updated.default_port_value = action_name;
  }
  return updated;
}

class MoveGripperTo : public BT::RosActionNode<gigatino_msgs::action::Move>
{
public:
  using Action = gigatino_msgs::action::Move;
  using Base = BT::RosActionNode<Action>;
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using WrappedResult = typename Base::WrappedResult;
  using Goal = typename Base::Goal;

  MoveGripperTo(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    : Base(name, config, withDefaultActionName(params, "gigatino/move"))
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("frame", "Frame in which x, y and z are expressed"),
        BT::InputPort<double>("x", "Target x in frame"),
        BT::InputPort<double>("y", "Target y in frame"),
        BT::InputPort<double>("z", "Target z in frame"),
        BT::InputPort<bool>("relative", false, "Whether the move is relative"),
        BT::InputPort<bool>("use_gripper", false, "Whether to command the gripper state"),
        BT::InputPort<bool>("gripper_state", false, "Requested gripper state"),
        BT::OutputPort<unsigned>("status_code", "gigatino status code"),
        BT::OutputPort<std::string>("message", "Result message from the action server"),
    });
  }

  bool setGoal(Goal& goal) override
  {
    goal.relative = getInput<bool>("relative").value_or(false);
    goal.x = static_cast<float>(getRequiredInput<double>("x"));
    goal.y = static_cast<float>(getRequiredInput<double>("y"));
    goal.z = static_cast<float>(getRequiredInput<double>("z"));
    auto node = Base::node_.lock();
    std::string namespace_name = node->get_namespace();
    namespace_name.erase(0, 1);
    goal.target_frame = namespace_name + "/" + getRequiredInput<std::string>("frame");
    goal.use_gripper = getInput<bool>("use_gripper").value_or(false);
    goal.gripper_state = getInput<bool>("gripper_state").value_or(false);
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    const auto status_code =
        result.result ? result.result->status_code : gigatino_msgs::msg::StatusCode::UNKNOWN;
    const auto message = result.result ? result.result->message : std::string{};

    setOutput("status_code", static_cast<unsigned>(status_code));
    setOutput("message", message);

    const bool success = status_code == gigatino_msgs::msg::StatusCode::SUCCESS;

    if(!success)
    {
      RCLCPP_ERROR(logger(), "%s completed with status_code=%u: %s", name().c_str(),
                   static_cast<unsigned>(status_code), message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult>& result) override
  {
    if(result && result->result)
    {
      setOutput("status_code", static_cast<unsigned>(result->result->status_code));
      setOutput("message", result->result->message);
      RCLCPP_ERROR(logger(), "%s failed: %s (%s)", name().c_str(), BT::toStr(error),
                   result->result->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), BT::toStr(error));
      setOutput("status_code", static_cast<unsigned>(gigatino_msgs::msg::StatusCode::UNKNOWN));
      setOutput("message", std::string{});
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  template <typename T>
  T getRequiredInput(const std::string& key)
  {
    auto value = getInput<T>(key);
    if(!value)
    {
      throw BT::RuntimeError("Missing required input port [", key, "] in ", name(), ": ",
                             value.error());
    }
    return value.value();
  }
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::MoveGripperTo, "MoveGripperTo");
