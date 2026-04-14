#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "motor_move_msgs/action/motor_move.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robotino_bahavior_tree/ros_action_bt_node.hpp"

namespace robotino_bahavior_tree
{

class MotorMove : public RosActionBtNode<motor_move_msgs::action::MotorMove>
{
public:
  using Action = motor_move_msgs::action::MotorMove;
  using Base = RosActionBtNode<Action>;
  using WrappedResult = typename Base::WrappedResult;

  MotorMove(const std::string & name, const BT::NodeConfiguration & config)
  : Base(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    auto ports = Base::providedPorts();
    ports.erase("server_name");
    ports.insert(BT::InputPort<std::string>(
        "server_name",
        "motor_move_action",
        "Action server name"));
    ports.insert(BT::InputPort<std::string>(
        "frame_id",
        "Frame in which x, y, z and yaw are expressed"));
    ports.insert(BT::InputPort<double>("x", "Target x position"));
    ports.insert(BT::InputPort<double>("y", "Target y position"));
    ports.insert(BT::InputPort<double>("yaw", "Target yaw in radians"));
    ports.insert(BT::InputPort<double>("qx", "Optional quaternion x override"));
    ports.insert(BT::InputPort<double>("qy", "Optional quaternion y override"));
    ports.insert(BT::InputPort<double>("qz", "Optional quaternion z override"));
    ports.insert(BT::InputPort<double>("qw", "Optional quaternion w override"));
    ports.insert(BT::OutputPort<double>(
        "distance_to_target",
        "Latest feedback distance to target"));
    return ports;
  }

protected:
  Action::Goal createGoal() override
  {
    Action::Goal goal;
    goal.motor_goal.header.stamp = node_->now();
    goal.motor_goal.header.frame_id = getRequiredInput<std::string>("frame_id");
    goal.motor_goal.pose.position.x = getRequiredInput<double>("x");
    goal.motor_goal.pose.position.y = getRequiredInput<double>("y");
    goal.motor_goal.pose.orientation = getOrientation();
    return goal;
  }

  void onFeedback(const Action::Feedback & feedback) override
  {
    last_distance_to_target_.store(feedback.distance_to_target);
  }

  void onFeedbackTick() override
  {
    setOutput("distance_to_target", last_distance_to_target_.load());
  }

  BT::NodeStatus onFailure(const std::string & reason) override
  {
    RCLCPP_ERROR(logger_, "%s", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onResult(const WrappedResult & result) override
  {
    const bool protocol_success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    const bool action_success = protocol_success && result.result && result.result->success;

    setOutput("success", action_success);
    setOutput("distance_to_target", last_distance_to_target_.load());

    if (!protocol_success) {
      return Base::onResult(result);
    }

    if (!action_success) {
      return onFailure(name() + " completed but reported success=false");
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  geometry_msgs::msg::Quaternion getOrientation()
  {
    const auto qx = getInput<double>("qx");
    const auto qy = getInput<double>("qy");
    const auto qz = getInput<double>("qz");
    const auto qw = getInput<double>("qw");

    const bool any_quaternion_input = static_cast<bool>(qx) || static_cast<bool>(qy) ||
      static_cast<bool>(qz) || static_cast<bool>(qw);
    const bool full_quaternion_input = static_cast<bool>(qx) && static_cast<bool>(qy) &&
      static_cast<bool>(qz) && static_cast<bool>(qw);

    if (any_quaternion_input && !full_quaternion_input) {
      throw BT::RuntimeError(
              "MotorMove requires qx, qy, qz and qw together when overriding yaw");
    }

    if (full_quaternion_input) {
      geometry_msgs::msg::Quaternion quat;
      quat.x = qx.value();
      quat.y = qy.value();
      quat.z = qz.value();
      quat.w = qw.value();
      return quat;
    }

    const auto yaw = getRequiredInput<double>("yaw");
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    quat.normalize();
    return tf2::toMsg(quat);
  }

  std::atomic<double> last_distance_to_target_{0.0};
};

}  // namespace robotino_bahavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robotino_bahavior_tree::MotorMove>("MotorMove");
}
