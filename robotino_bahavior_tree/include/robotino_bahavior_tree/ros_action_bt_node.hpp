#ifndef ROBOTINO_BAHAVIOR_TREE__ROS_ACTION_BT_NODE_HPP_
#define ROBOTINO_BAHAVIOR_TREE__ROS_ACTION_BT_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robotino_bahavior_tree
{

template<typename ActionT>
class RosActionBtNode : public BT::StatefulActionNode
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  RosActionBtNode(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    if (!config.blackboard) {
      throw BT::RuntimeError("Blackboard is required for node ", name);
    }

    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw BT::RuntimeError("Blackboard entry 'node' is missing for node ", name);
    }

    logger_ = node_->get_logger();
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "server_name",
        "Action server name"),
      BT::InputPort<double>(
        "server_timeout",
        1.0,
        "Timeout in seconds while waiting for the action server")
    };
  }

protected:
  template<typename T>
  T getRequiredInput(const std::string & key)
  {
    auto value = getInput<T>(key);
    if (!value) {
      throw BT::RuntimeError(
              "Missing required input port '", key, "' for node ", name(), ": ", value.error());
    }
    return value.value();
  }

  virtual typename ActionT::Goal createGoal() = 0;

  virtual void onFeedback(const typename ActionT::Feedback &)
  {
  }

  virtual void onFeedbackTick()
  {
  }

  virtual BT::NodeStatus onFailure(const std::string & reason)
  {
    RCLCPP_ERROR(logger_, "%s", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onResult(const WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return BT::NodeStatus::SUCCESS;
      case rclcpp_action::ResultCode::ABORTED:
        return onFailure(name() + " aborted by action server");
      case rclcpp_action::ResultCode::CANCELED:
        return onFailure(name() + " canceled");
      default:
        return onFailure(name() + " returned an unknown result code");
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("robotino_bahavior_tree")};

private:
  BT::NodeStatus onStart() override
  {
    goal_rejected_.store(false);
    result_available_.store(false);
    goal_active_.store(false);

    try {
      const auto server_name = getRequiredInput<std::string>("server_name");
      double server_timeout_s = 1.0;
      (void)getInput("server_timeout", server_timeout_s);

      ensureActionClient(server_name);
      if (!action_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_s))) {
        return onFailure(
          name() + " could not connect to action server '" + server_name +
          "' within " + std::to_string(server_timeout_s) + "s");
      }

      auto goal = createGoal();

      typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
      options.goal_response_callback =
        [this](typename GoalHandle::SharedPtr goal_handle)
        {
          if (!goal_handle) {
            goal_rejected_.store(true);
            return;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          goal_handle_ = std::move(goal_handle);
          goal_active_.store(true);
        };

      options.feedback_callback =
        [this](
        typename GoalHandle::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback)
        {
          if (!feedback) {
            return;
          }
          onFeedback(*feedback);
        };

      options.result_callback =
        [this](const WrappedResult & result)
        {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            result_ = result;
          }
          goal_active_.store(false);
          result_available_.store(true);
        };

      action_client_->async_send_goal(goal, options);
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception & ex) {
      return onFailure(name() + " failed to send goal: " + ex.what());
    }
  }

  BT::NodeStatus onRunning() override
  {
    onFeedbackTick();

    if (goal_rejected_.load()) {
      return onFailure(name() + " goal was rejected by the action server");
    }

    if (result_available_.load()) {
      WrappedResult result_copy;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_copy = result_;
      }
      return onResult(result_copy);
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_active_.load() && goal_handle_) {
      (void)action_client_->async_cancel_goal(goal_handle_);
    }
    goal_active_.store(false);
  }

  void ensureActionClient(const std::string & server_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (action_client_ && server_name == current_server_name_) {
      return;
    }

    action_client_ = rclcpp_action::create_client<ActionT>(node_, server_name);
    current_server_name_ = server_name;
  }

  std::mutex mutex_;
  std::string current_server_name_;
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  typename GoalHandle::SharedPtr goal_handle_;
  WrappedResult result_{};
  std::atomic<bool> goal_rejected_{false};
  std::atomic<bool> result_available_{false};
  std::atomic<bool> goal_active_{false};
};

}  // namespace robotino_bahavior_tree

#endif  // ROBOTINO_BAHAVIOR_TREE__ROS_ACTION_BT_NODE_HPP_
