#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h" // 使用同步ActionNode即可，因为服务调用通常很快
#include "controller_manager_msgs/srv/switch_controller.hpp"

// 一个简单的同步节点，用于调用服务
class SwitchController : public BT::SyncActionNode
{
public:
  SwitchController(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    // 从全局黑板获取Node句柄
    auto node_ptr = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_ptr) {
      throw std::runtime_error("Failed to get 'node' from the blackboard");
    }
    // 创建服务客户端
    client_ = node_ptr->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  }

  // 定义端口
  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::vector<std::string>>("activate", "Controllers to activate"),
      BT::InputPort<std::vector<std::string>>("deactivate", "Controllers to deactivate"),
      BT::InputPort<int>("strictness", 1, "Strictness level (1=STRICT, 2=BEST_EFFORT)")
    };
  }

  // tick()函数执行实际逻辑
  BT::NodeStatus tick() override
  {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("SwitchController_BT"), "Service not available");
      return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    
    // 从输入端口获取要激活和停用的控制器列表
    getInput("activate", request->activate_controllers);
    getInput("deactivate", request->deactivate_controllers);
    
    int strictness_val;
    getInput("strictness", strictness_val);
    request->strictness = strictness_val;
    
    request->start_asap = true; // 立即切换
    request->timeout.sec = 3; // 设置超时

    auto future = client_->async_send_request(request);

    // 等待服务调用完成
    if (rclcpp::spin_until_future_complete(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node"), 
        future, 
        std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SwitchController_BT"), "Failed to call service switch_controller");
      return BT::NodeStatus::FAILURE;
    }
    
    auto result = future.get();
    if (result->ok) {
      RCLCPP_INFO(rclcpp::get_logger("SwitchController_BT"), "Successfully switched controllers");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("SwitchController_BT"), "Failed to switch controllers");
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
};