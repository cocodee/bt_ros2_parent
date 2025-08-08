#ifndef BEHAVIOR_TREE_NODE_HPP_
#define BEHAVIOR_TREE_NODE_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "my_robot_interfaces/action/execute_command.hpp"
#include "bt_ros2/print_command_action.hpp"

namespace bt_ros2
{

class BehaviorTreeNode : public rclcpp::Node
{
public:
  explicit BehaviorTreeNode(const rclcpp::NodeOptions & options);

private:
  using ExecuteCommand = my_robot_interfaces::action::ExecuteCommand;
  using GoalHandleExecuteCommand = rclcpp_action::ServerGoalHandle<ExecuteCommand>;

  // Action Server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteCommand::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteCommand> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleExecuteCommand> goal_handle);

  // Behavior Tree main loop
  void tick_tree();

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::StdCoutLogger> bt_logger_;

  rclcpp_action::Server<ExecuteCommand>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GoalHandleExecuteCommand> current_goal_handle_;
};

} // namespace bt_ros2

#endif // BEHAVIOR_TREE_NODE_HPP_
