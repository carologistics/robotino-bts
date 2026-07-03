#include <cmath>
#include <cstdint>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "robotino_vision_msgs/action/acquire_object_tracking.hpp"

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

class AcquireObjectTracking
  : public BT::RosActionNode<robotino_vision_msgs::action::AcquireObjectTracking>
{
public:
  using Action = robotino_vision_msgs::action::AcquireObjectTracking;
  using Base = BT::RosActionNode<Action>;
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using WrappedResult = typename Base::WrappedResult;
  using Goal = typename Base::Goal;
  using Feedback = typename Base::Feedback;

  AcquireObjectTracking(
    const std::string& name, const NodeConfig& config, const RosNodeParams& params)
  : Base(name, config, withDefaultActionName(params, "acquire_object_tracking"))
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("object_prompt", "Text prompt describing the object to acquire"),
        BT::InputPort<std::string>("reference_frame", "Reference frame used for selecting the closest object"),
        BT::InputPort<double>("distance_threshold", "Maximum accepted object distance in meters"),
        BT::InputPort<double>("approach_distance", "Frozen approach TF standoff distance in meters"),
        BT::InputPort<double>("segmentation_confidence", "YOLO segmentation confidence threshold"),
        BT::InputPort<std::string>("target_color", "", "Optional target color filter"),
        BT::InputPort<std::string>("object_tf_name", "Frozen object TF frame name"),
        BT::InputPort<std::string>("approach_tf_name", "Frozen approach TF frame name"),
        BT::InputPort<double>("timeout_sec", "Maximum acquisition time in seconds"),
        BT::InputPort<unsigned>("min_stable_frames", "Required consecutive stable detections"),
        BT::InputPort<double>("max_position_jump", "Maximum object position jump between stable frames in meters"),
        BT::OutputPort<bool>("success", "Whether object acquisition succeeded"),
        BT::OutputPort<std::string>("error", "Error returned by object acquisition"),
        BT::OutputPort<std::string>("acquired_object_tf_name", "Resolved frozen object TF frame"),
        BT::OutputPort<std::string>("acquired_approach_tf_name", "Resolved frozen approach TF frame"),
        BT::OutputPort<bool>("detection_valid", "Feedback: current detection is valid"),
        BT::OutputPort<unsigned>("stable_frames", "Feedback: consecutive stable detection count"),
        BT::OutputPort<double>("object_distance", "Feedback: selected object distance in meters"),
    });
  }

  bool setGoal(Goal& goal) override
  {
    goal.object_prompt = requiredString("object_prompt");
    goal.reference_frame = requiredString("reference_frame");
    goal.distance_threshold = requiredPositiveDouble("distance_threshold");
    goal.approach_distance = requiredNonNegativeDouble("approach_distance");
    goal.segmentation_confidence = requiredUnitDouble("segmentation_confidence");
    goal.target_color = getInput<std::string>("target_color").value_or("");
    goal.object_tf_name = requiredString("object_tf_name");
    goal.approach_tf_name = requiredString("approach_tf_name");
    if(goal.object_tf_name == goal.approach_tf_name)
    {
      throw BT::RuntimeError(name(), ": object_tf_name and approach_tf_name must be different");
    }
    goal.timeout_sec = requiredPositiveDouble("timeout_sec");
    goal.min_stable_frames = static_cast<uint32_t>(requiredPositiveUnsigned("min_stable_frames"));
    goal.max_position_jump = requiredNonNegativeDouble("max_position_jump");

    RCLCPP_INFO(logger(),
                "%s sending goal: prompt='%s' reference_frame=%s distance_threshold=%.3f "
                "approach_distance=%.3f segmentation_confidence=%.3f target_color='%s' "
                "object_tf_name=%s approach_tf_name=%s timeout_sec=%.3f "
                "min_stable_frames=%u max_position_jump=%.3f",
                name().c_str(), goal.object_prompt.c_str(), goal.reference_frame.c_str(),
                goal.distance_threshold, goal.approach_distance, goal.segmentation_confidence,
                goal.target_color.c_str(), goal.object_tf_name.c_str(), goal.approach_tf_name.c_str(),
                goal.timeout_sec, goal.min_stable_frames, goal.max_position_jump);
    return true;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    if(feedback)
    {
      setOutput("detection_valid", feedback->detection_valid);
      setOutput("stable_frames", static_cast<unsigned>(feedback->stable_frames));
      setOutput("object_distance", feedback->object_distance);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& result) override
  {
    if(!result.result)
    {
      setOutput("success", false);
      setOutput("error", std::string{"missing action result"});
      RCLCPP_ERROR(logger(), "%s completed without result", name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    setOutput("success", result.result->success);
    setOutput("error", result.result->error);
    setOutput("acquired_object_tf_name", result.result->object_tf_name);
    setOutput("acquired_approach_tf_name", result.result->approach_tf_name);

    if(!result.result->success)
    {
      RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), result.result->error.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "%s completed successfully: object_tf=%s approach_tf=%s",
                name().c_str(), result.result->object_tf_name.c_str(),
                result.result->approach_tf_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult>& result) override
  {
    setOutput("success", false);
    if(result && result->result)
    {
      setOutput("error", result->result->error);
      RCLCPP_ERROR(logger(), "%s failed: %s (%s)", name().c_str(), BT::toStr(error),
                   result->result->error.c_str());
    }
    else
    {
      const std::string error_message = BT::toStr(error);
      setOutput("error", error_message);
      RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), error_message.c_str());
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

  std::string requiredString(const std::string& key)
  {
    auto value = getRequiredInput<std::string>(key);
    if(value.empty())
    {
      throw BT::RuntimeError(name(), ": input [", key, "] must not be empty");
    }
    return value;
  }

  double requiredFiniteDouble(const std::string& key)
  {
    const double value = getRequiredInput<double>(key);
    if(!std::isfinite(value))
    {
      throw BT::RuntimeError(name(), ": input [", key, "] must be finite");
    }
    return value;
  }

  double requiredPositiveDouble(const std::string& key)
  {
    const double value = requiredFiniteDouble(key);
    if(value <= 0.0)
    {
      throw BT::RuntimeError(name(), ": input [", key, "] must be > 0.0");
    }
    return value;
  }

  double requiredNonNegativeDouble(const std::string& key)
  {
    const double value = requiredFiniteDouble(key);
    if(value < 0.0)
    {
      throw BT::RuntimeError(name(), ": input [", key, "] must be >= 0.0");
    }
    return value;
  }

  double requiredUnitDouble(const std::string& key)
  {
    const double value = requiredFiniteDouble(key);
    if(value < 0.0 || value > 1.0)
    {
      throw BT::RuntimeError(name(), ": input [", key, "] must be in [0.0, 1.0]");
    }
    return value;
  }

  unsigned requiredPositiveUnsigned(const std::string& key)
  {
    const unsigned value = getRequiredInput<unsigned>(key);
    if(value == 0)
    {
      throw BT::RuntimeError(name(), ": input [", key, "] must be > 0");
    }
    return value;
  }
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::AcquireObjectTracking, "AcquireObjectTracking");
