#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "motor_move_msgs/action/move_to_shelf.hpp"

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

class GrayBoxAlign : public BT::RosActionNode<motor_move_msgs::action::MoveToShelf>
{
public:
  using Action = motor_move_msgs::action::MoveToShelf;
  using Base = BT::RosActionNode<Action>;
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using WrappedResult = typename Base::WrappedResult;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;

  GrayBoxAlign(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    : Base(name, config, withDefaultActionName(params, "gray_box_align_action"))
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<double>("timeout", 20.0, "Maximum gray-box alignment time in seconds"),
        BT::InputPort<double>("speed", 0.1, "Reserved for compatibility with MoveToShelf action goals"),
        BT::OutputPort<bool>("success", "Action-level success flag"),
        BT::OutputPort<std::string>("message", "Result message from the action server"),
        BT::OutputPort<double>("front_range", "Feedback front range"),
        BT::OutputPort<double>("elapsed_time", "Feedback elapsed time in seconds"),
    });
  }

  bool setGoal(Goal& goal) override
  {
    goal.timeout = static_cast<float>(getInput<double>("timeout").value_or(20.0));
    goal.speed = static_cast<float>(getInput<double>("speed").value_or(0.1));

    RCLCPP_INFO(logger(), "%s sending goal: timeout=%.3f", name().c_str(), goal.timeout);
    return true;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    if(feedback)
    {
      setOutput("front_range", static_cast<double>(feedback->front_range));
      setOutput("elapsed_time", static_cast<double>(feedback->elapsed_time));
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    const bool action_success = result.result && result.result->success;
    const auto message = result.result ? result.result->message : std::string{};

    setOutput("success", action_success);
    setOutput("message", message);

    if(!action_success)
    {
      RCLCPP_ERROR(logger(), "%s completed with success=false: %s", name().c_str(),
                   message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "%s completed successfully: %s", name().c_str(), message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult>& result) override
  {
    setOutput("success", false);
    if(result && result->result)
    {
      setOutput("message", result->result->message);
      RCLCPP_ERROR(logger(), "%s failed: %s (%s)", name().c_str(), BT::toStr(error),
                   result->result->message.c_str());
    }
    else
    {
      setOutput("message", std::string{});
      RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), BT::toStr(error));
    }
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::GrayBoxAlign, "GrayBoxAlign");
