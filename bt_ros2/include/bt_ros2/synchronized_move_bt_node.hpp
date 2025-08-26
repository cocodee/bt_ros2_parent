#include "nav2_behavior_tree/bt_action_node.hpp"
#include "my_robot_interfaces/action/synchronized_move.hpp" // 引用新接口

class SynchronizedMoveNode : public nav2_behavior_tree::BtActionNode<my_robot_interfaces::action::SynchronizedMove>
{
public:
  SynchronizedMoveNode(const std::string& name, const BT::NodeConfiguration& conf)
    : BtActionNode(name, "synchronized_move", conf) // "synchronized_move" 是Action名称
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<double>>("left_positions", "Target positions for the left arm"),
      BT::InputPort<std::vector<double>>("right_positions", "Target positions for the right arm"),
      BT::InputPort<int>("time_sec", "Time in seconds for the trajectory")
    };
  }

  void on_tick() override
  {
    getInput("left_positions", goal_.left_arm_positions);
    getInput("right_positions", goal_.right_arm_positions);
    int sec;
    getInput("time_sec", sec);
    goal_.time_from_start.sec = sec;

  }
};