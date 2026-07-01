#include <atomic>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "motor_move_msgs/action/motor_move.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

class MotorMove : public BT::RosActionNode<motor_move_msgs::action::MotorMove>
{
public:
  using Action = motor_move_msgs::action::MotorMove;
  using Base = BT::RosActionNode<Action>;
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using WrappedResult = typename Base::WrappedResult;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;

  MotorMove(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    : Base(name, config, withDefaultActionName(params, "motor_move_action"))
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("frame_id",
                                   "Frame in which x, y, z, and yaw are expressed"),
        BT::InputPort<double>("x", "Target x position"),
        BT::InputPort<double>("y", "Target y position"),
        BT::InputPort<double>("z", 0.0, "Target z position"),
        BT::InputPort<double>("yaw", "Target yaw in radians"),
        BT::InputPort<double>("qx", "Optional quaternion x override"),
        BT::InputPort<double>("qy", "Optional quaternion y override"),
        BT::InputPort<double>("qz", "Optional quaternion z override"),
        BT::InputPort<double>("qw", "Optional quaternion w override"),
        BT::OutputPort<double>("distance_to_target",
                               "Feedback distance to target"),
    });
  }

  bool setGoal(Goal& goal) override
  {
    // TODO: goal.motor_goal.header.stamp = now();
    auto node = Base::node_.lock();
    std::string namespace_name = node->get_namespace();
    namespace_name.erase(0, 1);
    goal.motor_goal.header.frame_id = namespace_name + "/" + getRequiredInput<std::string>("frame_id");
    goal.motor_goal.pose.position.x = getRequiredInput<double>("x");
    goal.motor_goal.pose.position.y = getRequiredInput<double>("y");
    goal.motor_goal.pose.position.z = getInput<double>("z").value_or(0.0);
    goal.motor_goal.pose.orientation = getOrientation();

    RCLCPP_INFO(logger(),
                "%s sending goal: frame_id=%s x=%.3f y=%.3f z=%.3f orientation=(%.3f, %.3f, %.3f, %.3f)",
                name().c_str(), goal.motor_goal.header.frame_id.c_str(),
                goal.motor_goal.pose.position.x, goal.motor_goal.pose.position.y,
                goal.motor_goal.pose.position.z,
                goal.motor_goal.pose.orientation.x, goal.motor_goal.pose.orientation.y,
                goal.motor_goal.pose.orientation.z, goal.motor_goal.pose.orientation.w);
    return true;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    if(feedback)
    {
      setOutput("distance_to_target", feedback->distance_to_target);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    const bool action_success = result.result && result.result->success;

    if(!action_success)
    {
      RCLCPP_ERROR(logger(), "%s completed with success=false", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult>& /*result*/) override
  {
    RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), BT::toStr(error));
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

  geometry_msgs::msg::Quaternion getOrientation()
  {
    auto qx = getInput<double>("qx");
    auto qy = getInput<double>("qy");
    auto qz = getInput<double>("qz");
    auto qw = getInput<double>("qw");

    const bool any_quaternion_input =
        static_cast<bool>(qx) || static_cast<bool>(qy) || static_cast<bool>(qz) ||
        static_cast<bool>(qw);
    const bool full_quaternion_input =
        static_cast<bool>(qx) && static_cast<bool>(qy) && static_cast<bool>(qz) &&
        static_cast<bool>(qw);

    if(any_quaternion_input && !full_quaternion_input)
    {
      throw BT::RuntimeError(
          "MotorMove requires qx, qy, qz and qw together when overriding yaw");
    }

    if(full_quaternion_input)
    {
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
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::MotorMove, "MotorMove");
