#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "gigatino_msgs/action/move.hpp"

#include "robotino_bahavior_tree/ros_action_bt_node.hpp"

namespace robotino_bahavior_tree
{

class MoveGripperTo : public RosActionBtNode<gigatino_msgs::action::Move>
{
public:
  using Action = gigatino_msgs::action::Move;
  using Base = RosActionBtNode<Action>;
  using WrappedResult = typename Base::WrappedResult;

  MoveGripperTo(const std::string & name, const BT::NodeConfiguration & config)
  : Base(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    auto ports = Base::providedPorts();
    ports.erase("server_name");
    ports.insert(BT::InputPort<std::string>(
        "server_name",
        "gigatino/move",
        "Action server name"));
    ports.insert(BT::InputPort<std::string>(
        "frame",
        "Frame in which x, y and z are expressed"));
    ports.insert(BT::InputPort<double>("x", "Target x in frame"));
    ports.insert(BT::InputPort<double>("y", "Target y in frame"));
    ports.insert(BT::InputPort<double>("z", "Target z in frame"));
    return ports;
  }

protected:
  Action::Goal createGoal() override
  {
    Action::Goal goal;
    goal.relative = false;
    goal.x = getRequiredInput<double>("x");
    goal.y = getRequiredInput<double>("y");
    goal.z = getRequiredInput<double>("z");
    goal.target_frame = getRequiredInput<std::string>("frame");
    return goal;
  }

  BT::NodeStatus onFailure(const std::string & reason) override
  {
    RCLCPP_ERROR(logger_, "%s", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onResult(const WrappedResult & result) override
  {
    const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED;

    if (!success) {
      return Base::onResult(result);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace robotino_bahavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robotino_bahavior_tree::MoveGripperTo>("MoveGripperTo");
}
