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
// 全局记忆锁与纯物理坐标校验器
// =========================================================
namespace GraspData {
    inline std::vector<double> locked_handle_pose;
    inline std::vector<double> locked_aruco_pose;

    inline bool check_joint_reached(const std::vector<double>& target, const fairino_msgs::msg::RobotNonrtState& state, double tol = 1.0) {
        if(target.size() < 6) return false;
        double err = std::max({
            std::abs(target[0] - state.j1_cur_pos), std::abs(target[1] - state.j2_cur_pos),
            std::abs(target[2] - state.j3_cur_pos), std::abs(target[3] - state.j4_cur_pos),
            std::abs(target[4] - state.j5_cur_pos), std::abs(target[5] - state.j6_cur_pos)
        });
        return err < tol;
    }

    inline bool check_cartesian_reached(const std::vector<double>& target, const fairino_msgs::msg::RobotNonrtState& state, double tol = 3.0) {
        if(target.size() < 3) return false;
        double dx = target[0] - state.cart_x_cur_pos;
        double dy = target[1] - state.cart_y_cur_pos;
        double dz = target[2] - state.cart_z_cur_pos;
        return std::sqrt(dx*dx + dy*dy + dz*dz) < tol;
    }
}

// =========================================================
// 1. 视觉监测节点 (V12 极简去冗版：严格 6 秒 EMA 收敛 + 无脑均值锁定)
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
// 2. 夹爪控制节点 (增加物理动作延时)
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
// 宏定义：终极物理闭环校验核
// =========================================================
#define CARTESIAN_CHECK_LOGIC(TOLERANCE) \
    rclcpp::spin_some(node_); \
    if (!latest_state_) return BT::NodeStatus::RUNNING; \
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - cmd_time_).count(); \
    bool reached = GraspData::check_cartesian_reached(target_pos_, *latest_state_, TOLERANCE); \
    if (!has_started_) { \
        if (latest_state_->robot_motion_done == 0) { has_started_ = true; RCLCPP_INFO(node_->get_logger(), "🚄 底层已起步..."); } \
        else if (elapsed > 2.0) { \
            if (reached) return BT::NodeStatus::SUCCESS; \
            RCLCPP_ERROR(node_->get_logger(), "🚨 熔断：未起步且不在目标！(目标Z:%.1f, 当前Z:%.1f)", target_pos_[2], latest_state_->cart_z_cur_pos); return BT::NodeStatus::FAILURE; \
        } \
        return BT::NodeStatus::RUNNING; \
    } else { \
        if (latest_state_->robot_motion_done == 1) { \
            if (reached) return BT::NodeStatus::SUCCESS; \
            RCLCPP_ERROR(node_->get_logger(), "💥 熔断：异常停机！疑似发生碰撞 (目标Z:%.1f, 实际Z:%.1f)", target_pos_[2], latest_state_->cart_z_cur_pos); return BT::NodeStatus::FAILURE; \
        } \
        return BT::NodeStatus::RUNNING; \
    }

#define JOINT_CHECK_LOGIC(TOLERANCE) \
    rclcpp::spin_some(node_); \
    if (!latest_state_) return BT::NodeStatus::RUNNING; \
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - cmd_time_).count(); \
    bool reached = GraspData::check_joint_reached(target_pos_, *latest_state_, TOLERANCE); \
    if (!has_started_) { \
        if (latest_state_->robot_motion_done == 0) { has_started_ = true; RCLCPP_INFO(node_->get_logger(), "🚄 底层已起步..."); } \
        else if (elapsed > 2.0) { \
            if (reached) return BT::NodeStatus::SUCCESS; \
            RCLCPP_ERROR(node_->get_logger(), "🚨 熔断：关节未响应！"); return BT::NodeStatus::FAILURE; \
        } \
        return BT::NodeStatus::RUNNING; \
    } else { \
        if (latest_state_->robot_motion_done == 1) { \
            if (reached) return BT::NodeStatus::SUCCESS; \
            RCLCPP_ERROR(node_->get_logger(), "💥 熔断：异常停机！关节角偏差过大！"); return BT::NodeStatus::FAILURE; \
        } \
        return BT::NodeStatus::RUNNING; \
    }

// =========================================================
// 3. 把手操作节点 (悬停 -> 慢插 -> 相对拔起)
// =========================================================
class MoveToLidApproach : public BT::StatefulActionNode {
public:
    MoveToLidApproach(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        if (GraspData::locked_handle_pose.empty()) return BT::NodeStatus::FAILURE;
        auto t = GraspData::locked_handle_pose; target_pos_ = {t[0], t[1], t[2] + 100.0}; // 在上方 10cm 悬停
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {2.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], t[3], t[4], t[5]}; 
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { CARTESIAN_CHECK_LOGIC(3.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

class MoveToLidGrasp : public BT::StatefulActionNode {
public:
    MoveToLidGrasp(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        if (GraspData::locked_handle_pose.empty()) return BT::NodeStatus::FAILURE;
        auto t = GraspData::locked_handle_pose; target_pos_ = {t[0], t[1], t[2] + 3.0}; // 留 8mm 余量，速度降为 5.0 往下直插
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {1.0, 5.0, target_pos_[0], target_pos_[1], target_pos_[2], t[3], t[4], t[5]}; 
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { CARTESIAN_CHECK_LOGIC(3.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

class MoveRelativeZUp150 : public BT::StatefulActionNode {
public:
    MoveRelativeZUp150(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        if (!latest_state_) return BT::NodeStatus::FAILURE;
        target_pos_ = {latest_state_->cart_x_cur_pos, latest_state_->cart_y_cur_pos, latest_state_->cart_z_cur_pos + 150.0};
        auto cmd = std_msgs::msg::Float64MultiArray(); 
        cmd.data = {1.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], latest_state_->cart_a_cur_pos, latest_state_->cart_b_cur_pos, latest_state_->cart_c_cur_pos};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { CARTESIAN_CHECK_LOGIC(3.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

// =========================================================
// 4. 绝对关节角盲操节点 (全量更新为最新点位，新增撤退点)
// =========================================================
class MoveToObserve : public BT::StatefulActionNode {
public:
    MoveToObserve(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        target_pos_ = {-123.093, -54.4, 69.633, -105.855, -91.203, 47.568};
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {3.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], target_pos_[3], target_pos_[4], target_pos_[5]};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { JOINT_CHECK_LOGIC(1.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

class MoveToLidDrop : public BT::StatefulActionNode {
public:
    MoveToLidDrop(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        target_pos_ = {-91.111, -49.863, 62.262, -107.5, -91.199, 47.568};
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {3.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], target_pos_[3], target_pos_[4], target_pos_[5]};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { JOINT_CHECK_LOGIC(1.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

// 【xh新加】绝对关节角撤退点
class MoveToLidDropRetreat : public BT::StatefulActionNode {
public:
    MoveToLidDropRetreat(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        target_pos_ = {-88.86, -58.578, 76.925, -109.098, -91.205, 47.568};
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {3.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], target_pos_[3], target_pos_[4], target_pos_[5]};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { JOINT_CHECK_LOGIC(1.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

class MoveToGunGrab : public BT::StatefulActionNode {
public:
    MoveToGunGrab(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        target_pos_ = {-101.182, -67.327, 93.768, -114.608, -91.624, 47.568};
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {3.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], target_pos_[3], target_pos_[4], target_pos_[5]};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { JOINT_CHECK_LOGIC(1.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

class MoveToPreInject : public BT::StatefulActionNode {
public:
    MoveToPreInject(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        target_pos_ = {-121.014, -42.733, 101.151, -236.295, -87.529, 47.567};
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {3.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], target_pos_[3], target_pos_[4], target_pos_[5]};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { JOINT_CHECK_LOGIC(1.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

// =========================================================
// 5. 注液核心
// =========================================================
class MoveToArUcoHover : public BT::StatefulActionNode {
public:
    MoveToArUcoHover(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        if (GraspData::locked_aruco_pose.empty()) return BT::NodeStatus::FAILURE;
        auto t = GraspData::locked_aruco_pose; target_pos_ = {t[0] + 90.0, t[1] - 110.0, t[2] + 350.0};
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {2.0, 15.0, target_pos_[0], target_pos_[1], target_pos_[2], 93.126, -49.220, -127.327};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override { CARTESIAN_CHECK_LOGIC(3.0) }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

class MoveToArUcoInject : public BT::StatefulActionNode {
public:
    MoveToArUcoInject(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) : BT::StatefulActionNode(name, config), node_(node) {
        pub_cmd_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_control_cmd", 10);
        sub_state_ = node_->create_subscription<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 10, [this](const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) { latest_state_ = msg; });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        if (GraspData::locked_aruco_pose.empty()) return BT::NodeStatus::FAILURE;
        auto t = GraspData::locked_aruco_pose; target_pos_ = {t[0] + 90.0, t[1] - 110.0, t[2] + 200.0};
        auto cmd = std_msgs::msg::Float64MultiArray(); cmd.data = {1.0, 5.0, target_pos_[0], target_pos_[1], target_pos_[2], 93.126, -49.220, -127.327};
        pub_cmd_->publish(cmd); cmd_time_ = std::chrono::steady_clock::now(); has_started_ = false; return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_);
        if (!latest_state_) return BT::NodeStatus::RUNNING;
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - cmd_time_).count();
        bool reached = GraspData::check_cartesian_reached(target_pos_, *latest_state_, 3.0);
        
        if (!has_started_) {
            if (latest_state_->robot_motion_done == 0) { has_started_ = true; }
            else if (elapsed > 2.0) {
                if (reached) { RCLCPP_INFO(node_->get_logger(), "💧 到位注液5秒..."); std::this_thread::sleep_for(std::chrono::seconds(5)); return BT::NodeStatus::SUCCESS; }
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        } else {
            if (latest_state_->robot_motion_done == 1) {
                if (reached) { RCLCPP_INFO(node_->get_logger(), "💧 插入到位，注液5秒..."); std::this_thread::sleep_for(std::chrono::seconds(5)); return BT::NodeStatus::SUCCESS; }
                RCLCPP_ERROR(node_->get_logger(), "💥 注液下探碰撞停机！"); return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
    }
    void onHalted() override {}
private: rclcpp::Node::SharedPtr node_; rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_; rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_; std::vector<double> target_pos_; bool has_started_ = false; std::chrono::steady_clock::time_point cmd_time_; fairino_msgs::msg::RobotNonrtState::SharedPtr latest_state_ = nullptr;
};

#endif // GRASP_ACTIONS_HPP