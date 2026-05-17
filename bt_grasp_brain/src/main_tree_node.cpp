#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h> 
#include <std_msgs/msg/string.hpp>                       
#include <std_msgs/msg/int8.hpp>  // 【新增】用于订阅底层反馈
#include "bt_grasp_brain/grasp_actions.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// =========================================================
// 核心黑科技：ROS 2 原生行为树状态遥测探针 (State Telemetry)
// =========================================================
class Ros2BTLogger : public BT::StatusChangeLogger {
public:
    Ros2BTLogger(BT::Tree& tree, rclcpp::Node::SharedPtr node)
        : BT::StatusChangeLogger(tree.rootNode()), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/bt_status", 10);
    }

    void callback(BT::Duration /*timestamp*/, const BT::TreeNode& node,
                  BT::NodeStatus /*prev_status*/, BT::NodeStatus status) override {
        
        std::string status_str = BT::toStr(status); 
        std::string type_str = (node.type() == BT::NodeType::ACTION) ? "ACTION" : 
                               (node.type() == BT::NodeType::CONDITION) ? "CONDITION" : "CONTROL";
        
        std::string json = "{\"node\": \"" + node.name() + "\", \"type\": \"" + type_str + "\", \"status\": \"" + status_str + "\"}";
        
        std_msgs::msg::String msg;
        msg.data = json;
        pub_->publish(msg); 
    }

    void flush() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_main_node");

    // ==========================================
    // 【核心新增】全局监听底层的战报反馈，实时写入共享内存
    // ==========================================
    auto feedback_sub = node->create_subscription<std_msgs::msg::Int8>(
        "/cmd_feedback", 10,
        [](const std_msgs::msg::Int8::SharedPtr msg) {
            GraspData::cmd_feedback_status = msg->data;
        });

    BT::BehaviorTreeFactory factory;

    // ==========================================
    // 1. 注册行为树节点 (极度精简版)
    // ==========================================
    // 视觉与夹爪节点
    factory.registerBuilder<CheckHandleVisible>("CheckHandleVisible", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<CheckHandleVisible>(name, config, node); });
    factory.registerBuilder<CheckArUcoVisible>("CheckArUcoVisible", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<CheckArUcoVisible>(name, config, node); });
    factory.registerBuilder<CloseGripper>("CloseGripper", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<CloseGripper>(name, config, node); });
    factory.registerBuilder<OpenGripper>("OpenGripper", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<OpenGripper>(name, config, node); });
    
    // 【革命性升级】万能运动节点，替代以前所有的死逻辑类
    factory.registerBuilder<MoveJAction>("MoveJAction", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveJAction>(name, config, node); });
    factory.registerBuilder<MoveLRelativeAction>("MoveLRelativeAction", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveLRelativeAction>(name, config, node); });

    // ==========================================
    // 2. 加载战术图纸
    // ==========================================
    std::string pkg_path = ament_index_cpp::get_package_share_directory("bt_grasp_brain");
    std::string xml_file = pkg_path + "/behavior_trees/main_task.xml";
    auto tree = factory.createTreeFromFile(xml_file);

    // ==========================================
    // 3. 挂载监控探针
    // ==========================================
    BT::PublisherZMQ publisher_zmq(tree);
    Ros2BTLogger ros2_logger(tree, node);

    RCLCPP_INFO(node->get_logger(), "🧠 终极行为树大脑 V8 启动！参数化架构部署完毕，等待指令...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(node->get_logger(), "🚀 通信握手完成，流水线开跑！");

    // ==========================================
    // 4. 驱动主循环
    // ==========================================
    rclcpp::Rate rate(10); 
    while (rclcpp::ok()) {
        BT::NodeStatus status = tree.tickRoot();
        
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "✅ 完美收工！全流程执行完毕！");
            break; 
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "❌ 异常熔断保护已生效！执行链中断。");
            break;
        }
        
        // 这里的 spin_some 极其重要，它驱动了 feedback_sub 的回调触发
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}