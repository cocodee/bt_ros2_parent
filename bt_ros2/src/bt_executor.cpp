#include "rclcpp/rclcpp.hpp"
#include "bt_ros2/behavior_tree_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<bt_ros2::BehaviorTreeNode>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
