#include <cmath>
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
        BT::InputPort<double>("x_min", 0.0, "Minimum accepted x target in meters"),
        BT::InputPort<double>("x_max", 0.245, "Maximum accepted x target in meters"),
        BT::InputPort<double>("z_min", 0.0, "Minimum accepted z target in meters"),
        BT::InputPort<double>("z_max", 0.145, "Maximum accepted z target in meters"),
        BT::InputPort<double>("upper_limit_factor", 1.5,
                              "Clamp values up to this factor of the axis range above max"),
        BT::InputPort<double>("lower_limit_fraction", 0.2,
                              "Clamp values up to this fraction of the axis range below min"),
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
    const double upper_limit_factor = getInput<double>("upper_limit_factor").value_or(1.5);
    const double lower_limit_fraction = getInput<double>("lower_limit_fraction").value_or(0.2);
    const double x = normalizeAxisInput("x", getRequiredInput<double>("x"),
                                        getInput<double>("x_min").value_or(0.0),
                                        getInput<double>("x_max").value_or(0.245),
                                        upper_limit_factor, lower_limit_fraction);
    const double z = normalizeAxisInput("z", getRequiredInput<double>("z"),
                                        getInput<double>("z_min").value_or(0.0),
                                        getInput<double>("z_max").value_or(0.145),
                                        upper_limit_factor, lower_limit_fraction);
    goal.x = static_cast<float>(x);
    goal.y = static_cast<float>(getRequiredInput<double>("y"));
    goal.z = static_cast<float>(z);
    auto node = Base::node_.lock();
    std::string namespace_name = node->get_namespace();
    namespace_name.erase(0, 1);
    goal.target_frame = namespace_name + "/" + getRequiredInput<std::string>("frame");
    goal.use_gripper = getInput<bool>("use_gripper").value_or(false);
    goal.gripper_state = getInput<bool>("gripper_state").value_or(false);

    RCLCPP_INFO(logger(),
                "%s sending goal: frame=%s x=%.3f y=%.3f z=%.3f relative=%s "
                "use_gripper=%s gripper_state=%s",
                name().c_str(), goal.target_frame.c_str(), goal.x, goal.y, goal.z,
                goal.relative ? "true" : "false",
                goal.use_gripper ? "true" : "false",
                goal.gripper_state ? "true" : "false");
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
    RCLCPP_INFO(logger(), "%s completed successfully: %s", name().c_str(), message.c_str());
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
  double normalizeAxisInput(const std::string& axis, double value, double min_value,
                            double max_value, double upper_limit_factor,
                            double lower_limit_fraction)
  {
    if(!std::isfinite(value))
    {
      const auto message = name() + ": input [" + axis + "] must be finite";
      RCLCPP_ERROR(logger(), "%s", message.c_str());
      throw BT::RuntimeError(message);
    }
    if(!std::isfinite(min_value) || !std::isfinite(max_value) || min_value >= max_value)
    {
      const auto message = name() + ": invalid limits for [" + axis + "]: min=" +
                           std::to_string(min_value) + " max=" + std::to_string(max_value);
      RCLCPP_ERROR(logger(), "%s", message.c_str());
      throw BT::RuntimeError(message);
    }
    if(!std::isfinite(upper_limit_factor) || upper_limit_factor < 1.0)
    {
      const auto message = name() + ": upper_limit_factor must be >= 1.0";
      RCLCPP_ERROR(logger(), "%s", message.c_str());
      throw BT::RuntimeError(message);
    }
    if(!std::isfinite(lower_limit_fraction) || lower_limit_fraction < 0.0)
    {
      const auto message = name() + ": lower_limit_fraction must be >= 0.0";
      RCLCPP_ERROR(logger(), "%s", message.c_str());
      throw BT::RuntimeError(message);
    }

    const double range = max_value - min_value;
    const double upper_clamp_limit = max_value + (upper_limit_factor - 1.0) * range;
    const double lower_clamp_limit = min_value - lower_limit_fraction * range;

    if(value > max_value)
    {
      if(value <= upper_clamp_limit)
      {
        RCLCPP_WARN(logger(),
                    "%s clamping %s from %.3f to max %.3f; allowed upper clamp limit is %.3f",
                    name().c_str(), axis.c_str(), value, max_value, upper_clamp_limit);
        return max_value;
      }
      const auto message = name() + ": input [" + axis + "]=" + std::to_string(value) +
                           " exceeds max " + std::to_string(max_value) +
                           " and allowed clamp limit " + std::to_string(upper_clamp_limit);
      RCLCPP_ERROR(logger(), "%s", message.c_str());
      throw BT::RuntimeError(message);
    }

    if(value < min_value)
    {
      if(value >= lower_clamp_limit)
      {
        RCLCPP_WARN(logger(),
                    "%s clamping %s from %.3f to min %.3f; allowed lower clamp limit is %.3f",
                    name().c_str(), axis.c_str(), value, min_value, lower_clamp_limit);
        return min_value;
      }
      const auto message = name() + ": input [" + axis + "]=" + std::to_string(value) +
                           " is below min " + std::to_string(min_value) +
                           " and allowed clamp limit " + std::to_string(lower_clamp_limit);
      RCLCPP_ERROR(logger(), "%s", message.c_str());
      throw BT::RuntimeError(message);
    }

    return value;
  }

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
