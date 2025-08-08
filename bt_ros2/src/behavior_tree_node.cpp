#include "bt_ros2/behavior_tree_node.hpp"

namespace bt_ros2
{

BehaviorTreeNode::BehaviorTreeNode(const rclcpp::NodeOptions & options)
  : Node("behavior_tree_node", options)
{
  // --- Behavior Tree Setup ---
  RCLCPP_INFO(this->get_logger(), "Setting up Behavior Tree...");

  // Register our custom node into the factory
  factory_.registerNodeType<PrintCommand>("PrintCommand");

  // Create the blackboard
  blackboard_ = BT::Blackboard::create();

  // Find the path to the XML file
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("bt_ros2");
  std::string xml_file = package_share_directory + "/bt_trees/print_command_tree.xml";

  // Create the tree. The blackboard is shared with the tree.
  tree_ = factory_.createTreeFromFile(xml_file, blackboard_);
  
  // Add a logger to print transitions on the console
  bt_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);

  // --- ROS 2 Action Server Setup ---
  RCLCPP_INFO(this->get_logger(), "Setting up Action Server...");
  this->action_server_ = rclcpp_action::create_server<ExecuteCommand>(
    this,
    "execute_command",
    std::bind(&BehaviorTreeNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&BehaviorTreeNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&BehaviorTreeNode::handle_accepted, this, std::placeholders::_1)
  );
  
  // --- Main Loop Timer ---
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), // Tick at 5 Hz
    std::bind(&BehaviorTreeNode::tick_tree, this)
  );
  RCLCPP_INFO(this->get_logger(), "Node initialized. Ready to receive commands.");
}

rclcpp_action::GoalResponse BehaviorTreeNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteCommand::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with command: %s", goal->command.c_str());
  // Always accept new goals
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BehaviorTreeNode::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteCommand> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  // Always accept cancel requests
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BehaviorTreeNode::handle_accepted(
  const std::shared_ptr<GoalHandleExecuteCommand> goal_handle)
{
  // This is where we link the action goal to the behavior tree
  RCLCPP_INFO(this->get_logger(), "Goal accepted. Writing command to blackboard.");
  
  // Store the goal handle
  current_goal_handle_ = goal_handle;

  // Halt any previously running tree to start fresh
  tree_.haltTree();

  // Write the command from the goal to the blackboard
  blackboard_->set("received_command", goal_handle->get_goal()->command);
}

void BehaviorTreeNode::tick_tree()
{
  if (!current_goal_handle_) {
    // No active goal, do nothing.
    return;
  }

  // Tick the tree and get the status
  BT::NodeStatus status = tree_.tickRoot();

  // Check the status to decide the action result
  if (status == BT::NodeStatus::SUCCESS) {
    auto result = std::make_shared<ExecuteCommand::Result>();
    result->success = true;
    current_goal_handle_->succeed(result);
    current_goal_handle_.reset(); // Clear the handle
    RCLCPP_INFO(this->get_logger(), "Behavior Tree executed successfully.");
  } else if (status == BT::NodeStatus::FAILURE) {
    auto result = std::make_shared<ExecuteCommand::Result>();
    result->success = false;
    current_goal_handle_->abort(result);
    current_goal_handle_.reset();
    RCLCPP_INFO(this->get_logger(), "Behavior Tree execution failed.");
  }
  // If status is RUNNING, do nothing and wait for the next tick.
}

} // namespace bt_ros2
