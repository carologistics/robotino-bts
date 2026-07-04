#include <cmath>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "gigatino_msgs/action/move.hpp"
#include "gigatino_msgs/msg/status_code.hpp"

namespace robotino_behavior_tree
{
namespace
{
constexpr double kTransformTimeoutSeconds = 0.1;
}


inline BT::RosNodeParams withDefaultActionName(const BT::RosNodeParams& params,
                                               const std::string& action_name)
{
  auto updated = params;
  if(updated.default_port_value.empty())
  {
    updated.default_port_value = action_name;
  }
  updated.server_timeout = std::chrono::seconds(3);
  updated.wait_for_server_timeout = std::chrono::seconds(3);
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
    auto node = Base::node_.lock();
    if(node)
    {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("frame", "Frame in which x, y and z are expressed"),
        BT::InputPort<double>("x", "Target x in frame"),
        BT::InputPort<double>("y", "Target y in frame"),
        BT::InputPort<double>("z", "Target z in frame"),
        BT::InputPort<std::string>(
            "z_override_in_end_effector_home", "",
            "Optional z target in end_effector_home after transforming absolute goals; empty disables"),
        BT::InputPort<double>("x_min", 0.0, "Minimum accepted x target in end_effector_home, meters"),
        BT::InputPort<double>("x_max", 0.19, "Maximum accepted x target in end_effector_home, meters"),
        BT::InputPort<double>("z_min", 0.0, "Minimum accepted z target in end_effector_home, meters"),
        BT::InputPort<double>("z_max", 0.14, "Maximum accepted z target in end_effector_home, meters"),
        BT::InputPort<double>("upper_limit_factor", 1.5,
                              "Clamp values up to this factor of the axis range above max"),
        BT::InputPort<double>("lower_limit_fraction", 0.2,
                              "Clamp values up to this fraction of the axis range below min"),
        BT::InputPort<bool>("relative", false, "Whether the move is relative"),
        BT::InputPort<bool>("use_gripper", false, "Whether to command the gripper state"),
        BT::InputPort<bool>("gripper_state", false, "Requested gripper state"),
        BT::InputPort<int>("timeout_ms", 3000,
                           "Maximum time to wait for the action to complete; 0 disables"),
        BT::OutputPort<unsigned>("status_code", "gigatino status code"),
        BT::OutputPort<std::string>("message", "Result message from the action server"),
    });
  }

  BT::NodeStatus tick() override
  {
    if(status() == BT::NodeStatus::IDLE)
    {
      timeout_ms_ = getInput<int>("timeout_ms").value_or(3000);
      if(timeout_ms_ < 0)
      {
        throw BT::RuntimeError("MoveGripperTo timeout_ms must be >= 0");
      }
      action_started_time_ = now();
    }

    const auto node_status = Base::tick();
    if(node_status == BT::NodeStatus::RUNNING && timeout_ms_ > 0)
    {
      const auto timeout =
          rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0);
      if((now() - action_started_time_) > timeout)
      {
        const auto message = name() + " timed out after " + std::to_string(timeout_ms_) + " ms";
        setOutput("status_code", static_cast<unsigned>(gigatino_msgs::msg::StatusCode::UNKNOWN));
        setOutput("message", message);
        RCLCPP_ERROR(logger(), "%s", message.c_str());
        Base::halt();
        resetStatus();
        return BT::NodeStatus::FAILURE;
      }
    }
    return node_status;
  }

  bool setGoal(Goal& goal) override
  {
    goal.relative = getInput<bool>("relative").value_or(false);
    const double raw_x = getRequiredInput<double>("x");
    const double raw_y = getRequiredInput<double>("y");
    const double raw_z = getRequiredInput<double>("z");
    const std::string requested_frame = namespacedFrame(getRequiredInput<std::string>("frame"));
    const std::string limit_frame = namespacedFrame("end_effector_home");
    const double upper_limit_factor = getInput<double>("upper_limit_factor").value_or(1.5);
    const double lower_limit_fraction = getInput<double>("lower_limit_fraction").value_or(0.2);
    const double x_min = getInput<double>("x_min").value_or(0.0);
    const double x_max = getInput<double>("x_max").value_or(0.19);
    const double z_min = getInput<double>("z_min").value_or(0.0);
    const double z_max = getInput<double>("z_max").value_or(0.14);
    const auto z_override_in_limit_frame = getOptionalDoubleInput("z_override_in_end_effector_home");

    double target_x = raw_x;
    double target_y = raw_y;
    double target_z = raw_z;
    std::string target_frame = requested_frame;

    if(goal.relative)
    {
      if(z_override_in_limit_frame)
      {
        throw BT::RuntimeError(name(),
                               ": z_override_in_end_effector_home is only valid for absolute moves");
      }
      target_x = normalizeAxisInput("x", raw_x, x_min, x_max, upper_limit_factor,
                                    lower_limit_fraction);
      target_z = normalizeAxisInput("z", raw_z, z_min, z_max, upper_limit_factor,
                                    lower_limit_fraction);
      RCLCPP_WARN(logger(),
                  "%s received relative gripper move; x/z limits are applied in the input frame %s",
                  name().c_str(), requested_frame.c_str());
    }
    else
    {
      geometry_msgs::msg::PoseStamped target_in_limit_frame;
      if(requested_frame == limit_frame)
      {
        target_in_limit_frame.header.frame_id = limit_frame;
        target_in_limit_frame.pose.position.x = raw_x;
        target_in_limit_frame.pose.position.y = raw_y;
        target_in_limit_frame.pose.position.z = raw_z;
        target_in_limit_frame.pose.orientation.w = 1.0;
        RCLCPP_INFO(logger(),
                    "%s using target already expressed in %s: (%.3f, %.3f, %.3f)",
                    name().c_str(), limit_frame.c_str(), raw_x, raw_y, raw_z);
      }
      else
      {
        target_in_limit_frame = transformTargetToFrame(
            requested_frame, raw_x, raw_y, raw_z, limit_frame);
        RCLCPP_INFO(logger(),
                    "%s transformed target from %s (%.3f, %.3f, %.3f) to %s "
                    "(%.3f, %.3f, %.3f) before z override and limit checks",
                    name().c_str(), requested_frame.c_str(), raw_x, raw_y, raw_z,
                    limit_frame.c_str(), target_in_limit_frame.pose.position.x,
                    target_in_limit_frame.pose.position.y,
                    target_in_limit_frame.pose.position.z);
      }

      double z_before_limits = target_in_limit_frame.pose.position.z;
      target_x = normalizeAxisInput("x", target_in_limit_frame.pose.position.x,
                                    x_min, x_max, upper_limit_factor,
                                    lower_limit_fraction);
      target_y = target_in_limit_frame.pose.position.y;
      if(z_override_in_limit_frame)
      {
        RCLCPP_INFO(logger(),
                    "%s overriding transformed z in %s from %.3f to %.3f before limit checks",
                    name().c_str(), limit_frame.c_str(), z_before_limits,
                    z_override_in_limit_frame.value());
        z_before_limits = z_override_in_limit_frame.value();
      }
      target_z = normalizeAxisInput("z", z_before_limits, z_min, z_max,
                                    upper_limit_factor, lower_limit_fraction);
      target_frame = limit_frame;
    }

    goal.x = static_cast<float>(target_x);
    goal.y = static_cast<float>(target_y);
    goal.z = static_cast<float>(target_z);
    goal.target_frame = target_frame;
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
  std::string namespacedFrame(std::string frame) const
  {
    while(!frame.empty() && frame.front() == '/')
    {
      frame.erase(frame.begin());
    }
    if(frame.empty())
    {
      throw BT::RuntimeError(name(), ": frame input must not be empty");
    }

    auto node = Base::node_.lock();
    if(!node)
    {
      throw BT::RuntimeError(name(), ": ROS node is not available");
    }
    std::string namespace_name = node->get_namespace();
    while(!namespace_name.empty() && namespace_name.front() == '/')
    {
      namespace_name.erase(namespace_name.begin());
    }
    if(namespace_name.empty() || frame == namespace_name || frame.rfind(namespace_name + "/", 0) == 0)
    {
      return frame;
    }
    return namespace_name + "/" + frame;
  }

  geometry_msgs::msg::PoseStamped transformTargetToFrame(
      const std::string& source_frame, double x, double y, double z,
      const std::string& target_frame)
  {
    if(!tf_buffer_)
    {
      throw BT::RuntimeError(name(), ": TF buffer is not available");
    }

    geometry_msgs::msg::PoseStamped source;
    source.header.frame_id = source_frame;
    source.header.stamp.sec = 0;
    source.header.stamp.nanosec = 0;
    source.pose.position.x = x;
    source.pose.position.y = y;
    source.pose.position.z = z;
    source.pose.orientation.w = 1.0;

    try
    {
      const auto transform = tf_buffer_->lookupTransform(
          target_frame, source_frame, tf2::TimePointZero,
          tf2::durationFromSec(kTransformTimeoutSeconds));
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(source, transformed, transform);
      return transformed;
    }
    catch(const tf2::TransformException& exc)
    {
      throw BT::RuntimeError(name(), ": cannot transform gripper target from ",
                             source_frame, " to ", target_frame, ": ", exc.what());
    }
  }

  std::optional<double> getOptionalDoubleInput(const std::string& key)
  {
    auto value_text = getInput<std::string>(key);
    if(!value_text)
    {
      throw BT::RuntimeError(name(), ": invalid optional input [", key, "]: ",
                             value_text.error());
    }
    if(value_text.value().empty())
    {
      return std::nullopt;
    }

    try
    {
      const double value = BT::convertFromString<double>(value_text.value());
      if(!std::isfinite(value))
      {
        throw BT::RuntimeError("value must be finite");
      }
      return value;
    }
    catch(const std::exception& exc)
    {
      throw BT::RuntimeError(name(), ": invalid optional input [", key, "]=",
                             value_text.value(), ": ", exc.what());
    }
  }

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

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time action_started_time_;
  int timeout_ms_ = 3000;
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::MoveGripperTo, "MoveGripperTo");
