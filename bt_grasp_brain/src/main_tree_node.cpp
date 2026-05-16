#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h> // 必须引入：行为树日志基类
#include <std_msgs/msg/string.hpp>                       // 必须引入：ROS 2 字符串消息
#include "bt_grasp_brain/grasp_actions.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// =========================================================
// 核心黑科技：ROS 2 原生行为树状态遥测探针 (State Telemetry)
// 作用：精准拦截每一颗节点的心跳，打包成纯净的 JSON 推送给 Web 上位机
// =========================================================
class Ros2BTLogger : public BT::StatusChangeLogger {
public:
    Ros2BTLogger(BT::Tree& tree, rclcpp::Node::SharedPtr node)
        : BT::StatusChangeLogger(tree.rootNode()), node_(node) {
        // 创建独立的状态发布话题
        pub_ = node_->create_publisher<std_msgs::msg::String>("/bt_status", 10);
    }

    // 【核心修正】：严格对齐 BehaviorTree.CPP V3 的传值签名 (BT::Duration)
    void callback(BT::Duration /*timestamp*/, const BT::TreeNode& node,
                  BT::NodeStatus /*prev_status*/, BT::NodeStatus status) override {
        
        std::string status_str = BT::toStr(status); // IDLE, RUNNING, SUCCESS, FAILURE
        std::string type_str = (node.type() == BT::NodeType::ACTION) ? "ACTION" : 
                               (node.type() == BT::NodeType::CONDITION) ? "CONDITION" : "CONTROL";
        
        // 组装极简工业级 JSON
        std::string json = "{\"node\": \"" + node.name() + "\", \"type\": \"" + type_str + "\", \"status\": \"" + status_str + "\"}";
        
        std_msgs::msg::String msg;
        msg.data = json;
        pub_->publish(msg); // 闪电推送给 Python 守护进程
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
    BT::BehaviorTreeFactory factory;

    // ==========================================
    // 1. 注册所有行为树节点
    // ==========================================
    factory.registerBuilder<CheckHandleVisible>("CheckHandleVisible", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<CheckHandleVisible>(name, config, node); });
    factory.registerBuilder<CheckArUcoVisible>("CheckArUcoVisible", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<CheckArUcoVisible>(name, config, node); });
    factory.registerBuilder<CloseGripper>("CloseGripper", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<CloseGripper>(name, config, node); });
    factory.registerBuilder<OpenGripper>("OpenGripper", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<OpenGripper>(name, config, node); });
    
    // 盖子操作核心
    factory.registerBuilder<MoveToLidApproach>("MoveToLidApproach", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToLidApproach>(name, config, node); });
    factory.registerBuilder<MoveToLidGrasp>("MoveToLidGrasp", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToLidGrasp>(name, config, node); });
    factory.registerBuilder<MoveRelativeZUp150>("MoveRelativeZUp150", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveRelativeZUp150>(name, config, node); });

    // 绝对关节角操作
    factory.registerBuilder<MoveToObserve>("MoveToObserve", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToObserve>(name, config, node); });
    factory.registerBuilder<MoveToLidDrop>("MoveToLidDrop", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToLidDrop>(name, config, node); });
    factory.registerBuilder<MoveToGunGrab>("MoveToGunGrab", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToGunGrab>(name, config, node); });
    factory.registerBuilder<MoveToLidDropRetreat>("MoveToLidDropRetreat", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToLidDropRetreat>(name, config, node); });
    factory.registerBuilder<MoveToPreInject>("MoveToPreInject", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToPreInject>(name, config, node); });
    
    // 注液操作
    factory.registerBuilder<MoveToArUcoHover>("MoveToArUcoHover", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToArUcoHover>(name, config, node); });
    factory.registerBuilder<MoveToArUcoInject>("MoveToArUcoInject", [&node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<MoveToArUcoInject>(name, config, node); });

    // ==========================================
    // 2. 加载战术图纸
    // ==========================================
    std::string pkg_path = ament_index_cpp::get_package_share_directory("bt_grasp_brain");
    std::string xml_file = pkg_path + "/behavior_trees/main_task.xml";
    auto tree = factory.createTreeFromFile(xml_file);

    // ==========================================
    // 3. 挂载监控探针
    // ==========================================
    // C++ 原生客户端监控器
    BT::PublisherZMQ publisher_zmq(tree);
    
    // 【核心植入】我们的 Web 遥测探针，死死盯住树的根节点
    Ros2BTLogger ros2_logger(tree, node);

    RCLCPP_INFO(node->get_logger(), "🧠 终极行为树大脑 V8 启动！状态遥测探针已挂载，等待指令...");
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
        
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}