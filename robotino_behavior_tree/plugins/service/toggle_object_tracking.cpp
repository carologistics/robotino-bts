#include <chrono>
#include <string>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "robotino_vision_msgs/srv/toggle_object_tracking.hpp"

namespace robotino_behavior_tree
{

inline BT::RosNodeParams withFixedServiceName(const BT::RosNodeParams & params)
{
  auto updated = params;
  updated.default_port_value = "object_tracking";
  updated.server_timeout = std::chrono::seconds(3);
  return updated;
}

class ToggleObjectTracking
  : public BT::RosServiceNode<robotino_vision_msgs::srv::ToggleObjectTracking>
{
public:
  using Service = robotino_vision_msgs::srv::ToggleObjectTracking;
  using Base = BT::RosServiceNode<Service>;
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using Request = typename Base::Request;
  using Response = typename Base::Response;

  ToggleObjectTracking(
    const std::string & name, const NodeConfig & config,
    const RosNodeParams & params)
  : Base(name, config, withFixedServiceName(params))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("enable", true, "Enable or disable object tracking"),
      BT::InputPort<std::string>("object_prompt", "grey eurobox EG 3212",
                                   "Text prompt describing the object to track"),
      BT::InputPort<std::string>("reference_frame", "base_link",
                                   "Reference frame used for object tracking"),
      BT::InputPort<double>("distance_threshold", 10.0,
                              "Maximum accepted object distance in meters"),
      BT::InputPort<double>("approach_distance", 0.0,
                              "Approach TF standoff distance in meters; 0 uses node default"),
      BT::InputPort<double>("segmentation_confidence", 0.2,
                              "YOLO segmentation confidence threshold"),
      BT::InputPort<std::string>("target_color", "",
                                   "Optional target color filter: green, blue, red, or yellow"),
      BT::InputPort<std::string>("object_tf_name", "target_object",
                                   "TF frame name published for the tracked object"),
      BT::OutputPort<bool>("success", "Whether the service accepted the request"),
      BT::OutputPort<std::string>("error", "Error returned by the service"),
    };
  }

  bool setRequest(Request::SharedPtr & request) override
  {
    request->enable = getInput<bool>("enable").value_or(true);
    request->object_prompt =
      getInput<std::string>("object_prompt").value_or("grey eurobox EG 3212");
    request->reference_frame =
      getInput<std::string>("reference_frame").value_or("base_link");
    request->distance_threshold = getInput<double>("distance_threshold").value_or(10.0);
    request->approach_distance = getInput<double>("approach_distance").value_or(0.0);
    request->segmentation_confidence =
      getInput<double>("segmentation_confidence").value_or(0.2);
    request->target_color = getInput<std::string>("target_color").value_or("");
    request->object_tf_name =
      getInput<std::string>("object_tf_name").value_or("target_object");

    RCLCPP_INFO(logger(),
                "%s sending request: enable=%s prompt='%s' reference_frame=%s "
                "distance_threshold=%.3f approach_distance=%.3f "
                "segmentation_confidence=%.3f target_color='%s' object_tf_name=%s",
                name().c_str(), request->enable ? "true" : "false",
                request->object_prompt.c_str(), request->reference_frame.c_str(),
                request->distance_threshold, request->approach_distance,
                request->segmentation_confidence, request->target_color.c_str(),
                request->object_tf_name.c_str());
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override
  {
    setOutput("success", response->success);
    setOutput("error", response->error);

    if(!response->success) {
      RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), response->error.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "%s completed successfully", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    const std::string error_message = BT::toStr(error);
    setOutput("success", false);
    setOutput("error", error_message);
    RCLCPP_ERROR(logger(), "%s failed: %s", name().c_str(), error_message.c_str());
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::ToggleObjectTracking, "ToggleObjectTracking");
