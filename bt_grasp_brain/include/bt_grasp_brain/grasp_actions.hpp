#ifndef GRASP_ACTIONS_HPP
#define GRASP_ACTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fairino_msgs/msg/robot_nonrt_state.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

// =========================================================
// 1.全局记忆锁与底层指令反馈监听
// =========================================================
namespace GraspData {
    inline std::vector<double> locked_handle_pose;
    inline std::vector<double> locked_aruco_pose;
    
    // 底层反馈状态：0=空闲/未知, 1=执行中, 2=成功, -1=失败或碰撞
    inline int cmd_feedback_status = 0; 
}

// =========================================================
// 2. 视觉监测节点 (V12 极简去冗版：严格 6 秒 EMA 收敛 + 无脑均值锁定)
// =========================================================
class CheckHandleVisible : public BT::StatefulActionNode {
public:
    CheckHandleVisible(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) 
        : BT::StatefulActionNode(name, config), node_(node) {
        
        // 必须使用 SensorDataQoS，强行向下兼容所有环境，防止假死
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(5);

        sub_status_ = node_->create_subscription<std_msgs::msg::Int8>(
            "/target_detected", qos, 
            [this](const std_msgs::msg::Int8::SharedPtr msg) { current_status_ = msg->data; });
        
        sub_pose_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/grasp_target_pose", qos, 
            [this, node](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { 
                if (!is_running_) return; // 没到我执行，坚决不吃数据
                
                auto now = std::chrono::steady_clock::now();
                // 绝对屏蔽期：静等 6 秒让机械臂彻底停稳，并等待 Python 端 EMA 滤波器完全收敛
                if (std::chrono::duration<double>(now - start_time_).count() < 6.0) return; 

                if (current_status_ == 2 && msg->data.size() >= 6) {
                    pose_buffer_.push_back(msg->data);
                    RCLCPP_INFO(node->get_logger(), "📥 实时截获把手帧... 进度: [%zu/5]", pose_buffer_.size());
                }
            });
    }
    static BT::PortsList providedPorts() { return {}; }
    
    BT::NodeStatus onStart() override {
        pose_buffer_.clear();
        start_time_ = std::chrono::steady_clock::now();
        is_running_ = true; 
        RCLCPP_INFO(node_->get_logger(), "👀 到达观测点！死等 6 秒，等待机械臂停稳及 EMA 算法收敛...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_);
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
        
        // 1. 绝对静默期拦截
        if (elapsed < 6.0) return BT::NodeStatus::RUNNING;

        // 2. 超时熔断拦截
        if (elapsed > 20.0) {
            RCLCPP_ERROR(node_->get_logger(), "🛑 超时熔断！6秒防抖结束后 14 秒内没凑齐 5 帧。当前仅有: %zu 帧", pose_buffer_.size());
            is_running_ = false;
            return BT::NodeStatus::FAILURE;
        }

        // 3. 极速放行：凑齐 5 帧直接计算均值，剥离所有冗余判断
        if (pose_buffer_.size() >= 5) {
            std::vector<double> avg_pose(6, 0.0);

            for (const auto& p : pose_buffer_) {
                for (int i = 0; i < 6; ++i) avg_pose[i] += p[i];
            }
            for (int i = 0; i < 6; ++i) avg_pose[i] /= pose_buffer_.size();

            GraspData::locked_handle_pose = avg_pose; 
            RCLCPP_INFO(node_->get_logger(), "✅ 6秒EMA收敛完毕！无波动判定，直接锁定 5 帧把手均值 Z=%.1f", avg_pose[2]);
            
            is_running_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    
    void onHalted() override { is_running_ = false; }

private: 
    rclcpp::Node::SharedPtr node_; rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_status_; rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pose_; int current_status_ = 0; std::vector<std::vector<double>> pose_buffer_; std::chrono::steady_clock::time_point start_time_; bool is_running_ = false;
};

class CheckArUcoVisible : public BT::StatefulActionNode {
public:
    CheckArUcoVisible(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) 
        : BT::StatefulActionNode(name, config), node_(node) {
        
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(10);

        sub_status_ = node_->create_subscription<std_msgs::msg::Int8>(
            "/target_detected", qos, 
            [this](const std_msgs::msg::Int8::SharedPtr msg) { current_status_ = msg->data; });
        
        sub_pose_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/grasp_target_pose", qos, 
            [this, node](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { 
                if (!is_running_) return;
                
                auto now = std::chrono::steady_clock::now();
                // 绝对屏蔽期：给相机和 ArUco 解算留足 6 秒稳定时间
                if (std::chrono::duration<double>(now - start_time_).count() < 6.0) return;

                if (current_status_ == 1 && msg->data.size() >= 6) {
                    pose_buffer_.push_back(msg->data);
                    RCLCPP_INFO(node->get_logger(), "📥 实时截获 ArUco 帧... 进度: [%zu/10]", pose_buffer_.size());
                }
            });
    }
    static BT::PortsList providedPorts() { return {}; }
    
    BT::NodeStatus onStart() override {
        pose_buffer_.clear();
        start_time_ = std::chrono::steady_clock::now();
        is_running_ = true;
        RCLCPP_INFO(node_->get_logger(), "👀 悬停到位！死等 6 秒，等待机械臂停稳及 EMA 算法收敛...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_);
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
        
        // 1. 绝对静默期拦截
        if (elapsed < 6.0) return BT::NodeStatus::RUNNING;

        // 2. 超时熔断拦截
        if (elapsed > 20.0) {
            RCLCPP_ERROR(node_->get_logger(), "🛑 超时熔断！6秒防抖结束后 14 秒内没凑齐 10 帧。当前仅有: %zu 帧", pose_buffer_.size());
            is_running_ = false;
            return BT::NodeStatus::FAILURE;
        }

        // 3. 极速放行：凑齐 10 帧直接计算均值
        if (pose_buffer_.size() >= 10) {
            std::vector<double> avg_pose(6, 0.0);

            for (const auto& p : pose_buffer_) {
                for (int i = 0; i < 6; ++i) avg_pose[i] += p[i];
            }
            for (int i = 0; i < 6; ++i) avg_pose[i] /= pose_buffer_.size();

            GraspData::locked_aruco_pose = avg_pose; 
            RCLCPP_INFO(node_->get_logger(), "✅ 6秒EMA收敛完毕！无波动判定，直接锁定 10 帧 ArUco 均值 Z=%.1f", avg_pose[2]);
            
            is_running_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    
    void onHalted() override { is_running_ = false; }

private: 
    rclcpp::Node::SharedPtr node_; rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_status_; rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pose_; int current_status_ = 0; std::vector<std::vector<double>> pose_buffer_; std::chrono::steady_clock::time_point start_time_; bool is_running_ = false;
};

// =========================================================
// 3. 夹爪控制节点 (增加物理动作延时)
// =========================================================
class CloseGripper : public BT::SyncActionNode {
public:
    CloseGripper(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::SyncActionNode(name, config), node_(node) { 
        pub_ = node_->create_publisher<std_msgs::msg::Int8>("/gripper_control_cmd", 10); 
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override { 
        std_msgs::msg::Int8 cmd; 
        cmd.data = 0; 
        pub_->publish(cmd); 
        RCLCPP_INFO(node_->get_logger(), "🗜️ 发送闭合指令，死等 2.5 秒确保彻底夹紧..."); 
        // 【核心修正】延时加长至 2500 毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(2500)); 
        return BT::NodeStatus::SUCCESS; 
    }
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_;
};

class OpenGripper : public BT::SyncActionNode {
public:
    OpenGripper(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::SyncActionNode(name, config), node_(node) { 
        pub_ = node_->create_publisher<std_msgs::msg::Int8>("/gripper_control_cmd", 10); 
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override { 
        std_msgs::msg::Int8 cmd; 
        cmd.data = 1; 
        pub_->publish(cmd); 
        RCLCPP_INFO(node_->get_logger(), "🗜️ 发送张开指令，死等 2.0 秒确保完全松脱..."); 
        // 【核心修正】延时加长至 2000 毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 
        return BT::NodeStatus::SUCCESS; 
    }
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_;
};

// =========================================================
// 4.万能绝对关节运动节点 (参数由 XML 动态注入)
// =========================================================
class MoveJAction : public BT::StatefulActionNode {
public:
    MoveJAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) 
        : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
    }

    // 暴露给 XML 的参数接口 (Ports)
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target"),  // 目标坐标字符串，例如 "-123.0, -54.4, ..."
            BT::InputPort<double>("speed"),        // 速度
            BT::InputPort<double>("tol")           // 动态容差
        };
    }

    BT::NodeStatus onStart() override {
        std::string target_str;
        double speed = 15.0;
        double tol = 1.0;

        // 从 XML 中读取参数，如果没写就用默认值
        if (!getInput<std::string>("target", target_str)) {
            RCLCPP_ERROR(node_->get_logger(), "未提供 target 参数！");
            return BT::NodeStatus::FAILURE;
        }
        getInput<double>("speed", speed);
        getInput<double>("tol", tol);

        // 解析以逗号分隔的字符串为 double 数组
        std::vector<double> target_pos;
        std::stringstream ss(target_str);
        std::string item;
        while (std::getline(ss, item, ',')) {
            target_pos.push_back(std::stod(item));
        }

        if (target_pos.size() != 6) {
            RCLCPP_ERROR(node_->get_logger(), "关节目标参数错误，必须是 6 个数值！");
            return BT::NodeStatus::FAILURE;
        }

        // 打包 9 位协议：[3.0(MoveJ类型), 速度, j1...j6, 容差]
        auto cmd = std_msgs::msg::Float64MultiArray(); 
        cmd.data = {3.0, speed, target_pos[0], target_pos[1], target_pos[2], target_pos[3], target_pos[4], target_pos[5], tol};
        
        GraspData::cmd_feedback_status = 0; // 重置反馈状态
        pub_cmd_->publish(cmd);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_); // 刷新回调

        // 纯粹的去算力化：只看底层给的信号
        if (GraspData::cmd_feedback_status == 2) {
            return BT::NodeStatus::SUCCESS;
        } else if (GraspData::cmd_feedback_status == -1) {
            RCLCPP_ERROR(node_->get_logger(), "🚨 底层反馈：动作执行失败或发生碰撞！");
            return BT::NodeStatus::FAILURE;
        }
        
        return BT::NodeStatus::RUNNING; // 如果是 0 或 1，继续等待
    }

    void onHalted() override {}

private: 
    rclcpp::Node::SharedPtr node_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
};

// =========================================================
// 5.万能相对笛卡尔直线运动 (支持基于 视觉目标 或 当前位置 的偏移)
// =========================================================
class MoveLRelativeAction : public BT::StatefulActionNode {
public:
    MoveLRelativeAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) 
        : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        // 保留状态订阅，仅用于 base_target == "current" 时的原点提取
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>(
            "nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }

    // 暴露给 XML 的核心参数
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("base_target"), // 参考点："handle", "aruco", 或 "current"
            BT::InputPort<double>("offset_x"),         // X轴偏移量
            BT::InputPort<double>("offset_y"),         // Y轴偏移量
            BT::InputPort<double>("offset_z"),         // Z轴偏移量
            BT::InputPort<double>("speed"),            // 运行速度
            BT::InputPort<double>("tol")               // 闭环容差
        };
    }

    BT::NodeStatus onStart() override {
        std::string base_target;
        double dx = 0.0, dy = 0.0, dz = 0.0, speed = 15.0, tol = 3.0;

        if (!getInput<std::string>("base_target", base_target)) {
            RCLCPP_ERROR(node_->get_logger(), "未提供 base_target 参数！");
            return BT::NodeStatus::FAILURE;
        }
        getInput<double>("offset_x", dx); getInput<double>("offset_y", dy); getInput<double>("offset_z", dz);
        getInput<double>("speed", speed); getInput<double>("tol", tol);

        std::vector<double> base_pose;
        double target_rx = 0.0, target_ry = 0.0, target_rz = 0.0;

        // =========================================================
        // 【核心修正】：姿态解耦逻辑
        // =========================================================
        if (base_target == "handle") {
            base_pose = GraspData::locked_handle_pose;
            if (base_pose.size() >= 6) {
                target_rx = base_pose[3]; target_ry = base_pose[4]; target_rz = base_pose[5]; // 把手：继承视觉姿态
            }
        } else if (base_target == "aruco") {
            base_pose = GraspData::locked_aruco_pose;
            if (!latest_state_) {
                RCLCPP_ERROR(node_->get_logger(), "未获取到底层状态，无法继承当前注液姿态！");
                return BT::NodeStatus::FAILURE;
            }
            // ⚠️ 强硬姿态拦截：ArUco通道只用 XYZ，姿态强行锁定为机械臂当前姿态！
            target_rx = latest_state_->cart_a_cur_pos; 
            target_ry = latest_state_->cart_b_cur_pos; 
            target_rz = latest_state_->cart_c_cur_pos;
        } else if (base_target == "current") {
            if (!latest_state_) return BT::NodeStatus::FAILURE;
            base_pose = {latest_state_->cart_x_cur_pos, latest_state_->cart_y_cur_pos, latest_state_->cart_z_cur_pos};
            target_rx = latest_state_->cart_a_cur_pos; 
            target_ry = latest_state_->cart_b_cur_pos; 
            target_rz = latest_state_->cart_c_cur_pos;
        } else {
            return BT::NodeStatus::FAILURE;
        }

        if (base_pose.empty() || base_pose.size() < 3) {
            RCLCPP_ERROR(node_->get_logger(), "参考点数据为空！");
            return BT::NodeStatus::FAILURE;
        }

        // 打包 9 位协议：完美融合视觉 XYZ 与 继承的 Rx, Ry, Rz
        auto cmd = std_msgs::msg::Float64MultiArray(); 
        cmd.data = {
            1.0, speed, 
            base_pose[0] + dx, base_pose[1] + dy, base_pose[2] + dz, 
            target_rx, target_ry, target_rz, 
            tol
        };
        
        GraspData::cmd_feedback_status = 0;
        pub_cmd_->publish(cmd);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_); // 刷新回调

        // 彻底去算力：无脑信任 control_node 传回来的校验结果
        if (GraspData::cmd_feedback_status == 2) {
            return BT::NodeStatus::SUCCESS;
        } else if (GraspData::cmd_feedback_status == -1) {
            RCLCPP_ERROR(node_->get_logger(), "🚨 底层反馈：动作执行失败或发生碰撞！");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private: 
    rclcpp::Node::SharedPtr node_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_;
    fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

#endif // GRASP_ACTIONS_HPP