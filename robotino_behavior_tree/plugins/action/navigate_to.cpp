#include <atomic>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
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

class NavigateTo : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using Base = BT::RosActionNode<Action>;
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using WrappedResult = typename Base::WrappedResult;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;

  NavigateTo(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    : Base(name, config, withDefaultActionName(params, "navigate_to_pose"))
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("frame_id",
                                   "Frame in which x, y, and yaw are expressed"),
        BT::InputPort<double>("x", "Target x position"),
        BT::InputPort<double>("y", "Target y position"),
        BT::InputPort<double>("yaw", "Target yaw in radians"),
        BT::InputPort<double>("qx", "Optional quaternion x override"),
        BT::InputPort<double>("qy", "Optional quaternion y override"),
        BT::InputPort<double>("qz", "Optional quaternion z override"),
        BT::InputPort<double>("qw", "Optional quaternion w override"),
        BT::InputPort<std::string>("behavior_tree", "Optional which behavior_tree to load"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose",
                               "Feedback current_pose"),
        BT::OutputPort<rclcpp::Duration>("navigation_time",
                               "Feedback time spent on navigation"),
        BT::OutputPort<rclcpp::Duration>("estimated_time_remaining",
                               "Feedback time remaining to target"),
        BT::OutputPort<int16_t>("number_of_recoveries",
                               "Feedback number of recovery maneuvers"),
        BT::OutputPort<float>("distance_remaining",
                               "Feedback distance remaining to target"),
        BT::OutputPort<uint16_t>("error_code",
                               "Result: See nav2_msgs/action/NavigateToPose"),
        BT::OutputPort<std::string>("error_msg",
                               "Result error message from nav2"),
    });
  }

  bool setGoal(Goal& goal) override
  {
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = getRequiredInput<std::string>("frame_id");
    goal.pose.pose.position.x = getRequiredInput<double>("x");
    goal.pose.pose.position.y = getRequiredInput<double>("y");
    goal.pose.pose.orientation = getOrientation();
    goal.behavior_tree = getInput<std::string>("behavior_tree").value_or("");
    return true;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    if(feedback)
    {
      setOutput("current_pose", feedback->current_pose);
      setOutput("navigation_time", feedback->navigation_time);
      setOutput("estimated_time_remaining", feedback->estimated_time_remaining);
      setOutput("number_of_recoveries", feedback->number_of_recoveries);
      setOutput("distance_remaining", feedback->distance_remaining);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    if(!result.result) {
      RCLCPP_ERROR(logger(), "%s completed without result", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    const uint16_t error_code = result.result->error_code;
    setOutput("error_code", error_code);
    setOutput("error_msg", result.result->error_msg);

    if(error_code != 0)
    {
      RCLCPP_ERROR(logger(), "%s completed with error_code=%u, message=%s",
                   name().c_str(), error_code, result.result->error_msg.c_str());
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
          "NavigateTo requires qx, qy, qz and qw together when overriding yaw");
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

CreateRosNodePlugin(robotino_behavior_tree::NavigateTo, "NavigateTo");
