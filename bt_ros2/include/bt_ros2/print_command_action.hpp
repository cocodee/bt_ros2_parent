#ifndef PRINT_COMMAND_ACTION_HPP_
#define PRINT_COMMAND_ACTION_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

// Simple synchronous action that prints a message from its input port.
class PrintCommand : public BT::SyncActionNode
{
public:
  // The constructor expects a node name and the configuration.
  PrintCommand(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  // Define the input ports required by this node.
  static BT::PortsList providedPorts()
  {
    // This node has a single input port called "message"
    return { BT::InputPort<std::string>("message") };
  }

  // This is the method that is executed by the tree.
  BT::NodeStatus tick() override
  {
    // Retrieve the message from the input port.
    auto res = getInput<std::string>("message");
    if (!res) {
      // If the input port is not set, throw an error.
      throw BT::RuntimeError("missing required input [message]: ", res.error());
    }

    // Print the message to the console.
    // In a real ROS 2 node, you should use RCLCPP_INFO.
    // Here we use std::cout for simplicity.
    std::cout << "[PrintCommand]: " << res.value() << std::endl;
    
    // The action is always successful.
    return BT::NodeStatus::SUCCESS;
  }
};

#endif // PRINT_COMMAND_ACTION_HPP_
