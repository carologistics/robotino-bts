#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "gigatino_msgs/msg/feedback.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotino_behavior_tree
{

class IsGripperReferenced : public BT::ConditionNode
{
public:
  using NodeConfig = BT::NodeConfig;
  using RosNodeParams = BT::RosNodeParams;
  using Feedback = gigatino_msgs::msg::Feedback;

  IsGripperReferenced(const std::string& name, const NodeConfig& config,
                      const RosNodeParams& params)
    : BT::ConditionNode(name, config), node_(params.nh.lock())
  {
    if(!node_)
    {
      throw BT::RuntimeError("ROS node expired while creating IsGripperReferenced");
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("topic_name", "gigatino/feedback",
                                   "Feedback topic containing the referenced flag"),
        BT::OutputPort<bool>("referenced", "Latest gigatino referenced state"),
    };
  }

  BT::NodeStatus tick() override
  {
    ensureSubscription();

    std::optional<bool> referenced;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(last_feedback_)
      {
        referenced = last_feedback_->referenced;
      }
    }

    if(!referenced)
    {
      setOutput("referenced", false);
      if(!reported_missing_feedback_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "%s has not received gripper feedback yet; treating gripper as unreferenced",
                    name().c_str());
        reported_missing_feedback_ = true;
      }
      return BT::NodeStatus::FAILURE;
    }

    reported_missing_feedback_ = false;
    setOutput("referenced", referenced.value());

    if(!last_reported_referenced_ || *last_reported_referenced_ != referenced.value())
    {
      last_reported_referenced_ = referenced.value();
      if(referenced.value())
      {
        RCLCPP_INFO(node_->get_logger(), "%s: gripper is referenced", name().c_str());
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "%s: gripper is not referenced", name().c_str());
      }
    }

    return referenced.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  void ensureSubscription()
  {
    if(subscription_)
    {
      return;
    }

    const auto topic_name = getInput<std::string>("topic_name").value_or("gigatino/feedback");
    subscription_ = node_->create_subscription<Feedback>(
        topic_name, rclcpp::QoS(1).best_effort(),
        [this](Feedback::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          last_feedback_ = msg;
        });

    RCLCPP_INFO(node_->get_logger(), "%s subscribed to %s", name().c_str(),
                topic_name.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Feedback>::SharedPtr subscription_;
  std::mutex mutex_;
  Feedback::SharedPtr last_feedback_;
  bool reported_missing_feedback_ = false;
  std::optional<bool> last_reported_referenced_;
};

}  // namespace robotino_behavior_tree

CreateRosNodePlugin(robotino_behavior_tree::IsGripperReferenced, "IsGripperReferenced");
