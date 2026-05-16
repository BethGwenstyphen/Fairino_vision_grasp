#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "fairino_msgs/msg/robot_nonrt_state.hpp"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        // 1. 声明动态安全参数
        this->declare_parameter("col_level_1", 5.0);
        this->declare_parameter("col_level_2", 5.0);
        this->declare_parameter("col_level_3", 5.0);
        this->declare_parameter("col_level_4", 5.0);
        this->declare_parameter("col_level_5", 5.0);
        this->declare_parameter("col_level_6", 5.0);
        this->declare_parameter("col_strategy", 0);
        this->declare_parameter("pos_tolerance", 2.0);
        this->declare_parameter("jnt_tolerance", 1.0);

        // 注册参数动态修改回调 (已修复 unused parameter 警告)
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        param_callback_handle_ = param_subscriber_->add_parameter_callback(
            "col_level_1", [this](const rclcpp::Parameter & /*p*/) { update_collision_settings(); });

        // 2. 建立服务客户端
        client_ = this->create_client<fairino_msgs::srv::RemoteCmdInterface>("/fairino_remote_command_service");
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "客户端被中断，等待服务失败。");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "⏳ 等待机器人命令服务启动...");
        }

        // 3. 核心闭环状态机变量初始化
        motion_done_ = true;
        exec_state_ = ExecState::IDLE;
        cmd_sent_time_ = this->now();
        move_type_ = 0;
        cur_cart_ = {0.0, 0.0, 0.0};
        cur_jnt_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_gripper_state_ = -1;
        default_movel_speed_ = 15.0;
        default_movej_speed_ = 10.0;

        // 4. 订阅与发布
        state_sub_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>(
            "nonrt_state_data", 10, std::bind(&ControlNode::state_callback, this, std::placeholders::_1));
        
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm_control_cmd", 10, std::bind(&ControlNode::control_callback, this, std::placeholders::_1));
        
        gripper_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/gripper_control_cmd", 10, std::bind(&ControlNode::gripper_callback, this, std::placeholders::_1));

        gripper_pub_ = this->create_publisher<std_msgs::msg::Int8>("/gripper_status", 10);
        feedback_pub_ = this->create_publisher<std_msgs::msg::Int8>("/cmd_feedback", 10);

        init_robot();

        // =========================================================
        // 【核心修改】启动后台独立线程：强行让队列处理循环跑起来
        // =========================================================
        cmd_thread_ = std::thread(&ControlNode::cmd_process_loop, this);

        RCLCPP_INFO(this->get_logger(), "✅ 工业级控制节点 V4 (闭环校验+动态碰撞保护) 已就绪！[C++重构版]");
    }

    // =========================================================
    // 【核心修改】新增析构函数：系统关闭时，强硬回收后台线程，防止Core Dump
    // =========================================================
    ~ControlNode()
    {
        running_ = false; // 改变标志位，让后台循环退出
        if (cmd_thread_.joinable()) {
            cmd_thread_.join(); // 死等后台线程完全退出，再释放整个节点内存
        }
    }

private:
    enum class ExecState { IDLE, WAIT_START, MOVING };

    std::queue<std::string> cmd_queue_;
    std::mutex queue_mutex_;
    std::thread cmd_thread_;
    std::atomic<bool> running_{true}; // 必须引入头文件 #include <atomic>

    bool motion_done_;
    ExecState exec_state_;
    rclcpp::Time cmd_sent_time_;
    int move_type_;
    std::vector<double> target_data_;
    std::vector<double> cur_cart_;
    std::vector<double> cur_jnt_;
    int current_gripper_state_;
    double default_movel_speed_;
    double default_movej_speed_;
    double cmd_tolerance_ = 2.0;

    rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr client_;
    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr feedback_pub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    rclcpp::ParameterCallbackHandle::SharedPtr param_callback_handle_;

    void call_service(const std::string& cmd_str) {
        // 仅仅将指令推入队列，不阻塞回调函数
        std::lock_guard<std::mutex> lock(queue_mutex_);
        cmd_queue_.push(cmd_str);
    }

    void cmd_process_loop() {
        while(rclcpp::ok() && running_) {
            std::string cmd = "";
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if(!cmd_queue_.empty()) {
                    cmd = cmd_queue_.front();
                    cmd_queue_.pop();
                }
            }
            
            if(!cmd.empty()) {
                cmd.erase(std::remove(cmd.begin(), cmd.end(), ' '), cmd.end());
                auto req = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
                req->cmd_str = cmd;
                
                // 阻塞等待服务结果，保证上一条指令底层确认后，再发下一条
                auto result_future = client_->async_send_request(req);
                // 这里阻塞在独立的线程中，绝对不会卡死主 ROS 节点的心跳！
                result_future.wait(); 
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void update_collision_settings()
    {
        double l1 = this->get_parameter("col_level_1").as_double();
        double l2 = this->get_parameter("col_level_2").as_double();
        double l3 = this->get_parameter("col_level_3").as_double();
        double l4 = this->get_parameter("col_level_4").as_double();
        double l5 = this->get_parameter("col_level_5").as_double();
        double l6 = this->get_parameter("col_level_6").as_double();
        int strategy = this->get_parameter("col_strategy").as_int();

        std::stringstream ss_col;
        ss_col << "SetAnticollision(" << l1 << "," << l2 << "," << l3 << "," << l4 << "," << l5 << "," << l6 << ")";
        call_service(ss_col.str());

        std::stringstream ss_str;
        ss_str << "SetCollisionStrategy(" << strategy << ")";
        call_service(ss_str.str());

        RCLCPP_INFO(this->get_logger(), "🛡️ 碰撞防护更新: 等级[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f], 策略[%d]", 
                    l1, l2, l3, l4, l5, l6, strategy);
    }

    void init_robot()
    {
        RCLCPP_INFO(this->get_logger(), "🤖 初始化硬件状态...");
        call_service("ResetAllError()");
        call_service("RobotEnable(1)");
        call_service("Mode(0)");      
        call_service("SetSpeed(20)"); 
        call_service("SetToolCoord(1,0,0,0,0,0,0)");
        call_service("SetWObjCoord(1,0,0,0,0,0,0)");
        
        update_collision_settings();
        
        RCLCPP_INFO(this->get_logger(), "🗜️ 默认初始化夹爪为张开状态...");
        call_service("SetToolDO(0,0)");
        call_service("SetToolDO(1,1)");
        current_gripper_state_ = 1;
        
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        gripper_pub_->publish(msg);
    }

    void state_callback(const fairino_msgs::msg::RobotNonrtState::SharedPtr msg)
    {
        motion_done_ = (msg->robot_motion_done != 0);
        
        cur_cart_ = {msg->cart_x_cur_pos, msg->cart_y_cur_pos, msg->cart_z_cur_pos};
        cur_jnt_ = {msg->j1_cur_pos, msg->j2_cur_pos, msg->j3_cur_pos, msg->j4_cur_pos, msg->j5_cur_pos, msg->j6_cur_pos};

        if (exec_state_ == ExecState::IDLE) return;

        double elapsed = (this->now() - cmd_sent_time_).seconds();

        if (exec_state_ == ExecState::WAIT_START) {
            if (!motion_done_) {
                exec_state_ = ExecState::MOVING;
            } else if (elapsed > 2.0) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 超过 2 秒未检测到起步，强制进行终点校验...");
                verify_target_and_unlock();
            }
        }
        else if (exec_state_ == ExecState::MOVING) {
            if (motion_done_) {
                verify_target_and_unlock();
            }
        }
    }

    void verify_target_and_unlock()
    {
        bool success = false;
        
        if (move_type_ == 1 || move_type_ == 2) {
            double dist_error = std::sqrt(
                std::pow(cur_cart_[0] - target_data_[0], 2) +
                std::pow(cur_cart_[1] - target_data_[1], 2) +
                std::pow(cur_cart_[2] - target_data_[2], 2)
            );
            double tol = cmd_tolerance_;
            // double tol = this->get_parameter("pos_tolerance").as_double();
            
            if (dist_error <= tol) {
                success = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "🛑 位置校验失败！距目标点偏差 %.2fmm (容差 %.1fmm)", dist_error, tol);
            }
        }
        else if (move_type_ == 3) {
            double max_error = 0.0;
            for(size_t i=0; i<6; ++i) {
                double err = std::abs(cur_jnt_[i] - target_data_[i]);
                if(err > max_error) max_error = err;
            }
            double tol = cmd_tolerance_;
            // double tol = this->get_parameter("jnt_tolerance").as_double();
            
            if (max_error <= tol) {
                success = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "🛑 关节校验失败！距目标角度最大偏差 %.2f° (容差 %.1f°)", max_error, tol);
            }
        }

        std_msgs::msg::Int8 fb_msg;
        if (success) {
            RCLCPP_INFO(this->get_logger(), "🎯 到达目标，闭环位置校验通过！");
            fb_msg.data = 2;
        } else {
            RCLCPP_ERROR(this->get_logger(), "⚠️ 动作可能被强制中断或发生碰撞！");
            fb_msg.data = -1;
        }
        feedback_pub_->publish(fb_msg);
        exec_state_ = ExecState::IDLE; 
    }

    void gripper_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        int cmd = msg->data;
        std_msgs::msg::Int8 pub_msg;
        if (cmd == 1) {
            RCLCPP_INFO(this->get_logger(), "🗜️ 张开夹爪");
            call_service("SetToolDO(0,0)"); 
            call_service("SetToolDO(1,1)"); 
            current_gripper_state_ = 1;
            pub_msg.data = 1;
            gripper_pub_->publish(pub_msg);
        } else if (cmd == 0) {
            RCLCPP_INFO(this->get_logger(), "🗜️ 闭合夹爪");
            call_service("SetToolDO(1,0)"); 
            call_service("SetToolDO(0,1)"); 
            current_gripper_state_ = 0;
            pub_msg.data = 0;
            gripper_pub_->publish(pub_msg);
        }
    }

    void control_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (exec_state_ != ExecState::IDLE) {
            RCLCPP_WARN(this->get_logger(), "✋ 机器人忙碌中，新指令已被拒绝。");
            std_msgs::msg::Int8 fb_msg;
            fb_msg.data = 0;
            feedback_pub_->publish(fb_msg);
            return;
        }

        if (msg->data.size() < 9) {
            RCLCPP_ERROR(this->get_logger(), "❌ 数据长度不足，协议需 9 位: [类型, 速度, X, Y, Z, Rx, Ry, Rz, 容差]");
            return;
        }

        move_type_ = static_cast<int>(msg->data[0]);
        double input_speed = msg->data[1];
        
        target_data_.clear();
        for(size_t i=2; i<8; ++i) target_data_.push_back(msg->data[i]);

        cmd_tolerance_ = msg->data[8];

        exec_state_ = ExecState::WAIT_START;
        cmd_sent_time_ = this->now();
        
        std_msgs::msg::Int8 fb_msg;
        fb_msg.data = 1;
        feedback_pub_->publish(fb_msg);

        const auto& p = target_data_;
        char buf1[256];
        char buf2[256];

        if (move_type_ == 1) { 
            double speed = input_speed > 0 ? input_speed : default_movel_speed_;
            snprintf(buf1, sizeof(buf1), "CARTPoint(1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)", p[0], p[1], p[2], p[3], p[4], p[5]);
            snprintf(buf2, sizeof(buf2), "MoveL(CART1,%.2f,1,1)", speed);
            call_service(buf1);
            RCLCPP_INFO(this->get_logger(), "🚀 MoveL -> CART1, 速度: %.2f", speed);
            call_service(buf2);
        }
        else if (move_type_ == 2) { 
            double speed = input_speed > 0 ? input_speed : default_movej_speed_;
            snprintf(buf1, sizeof(buf1), "CARTPoint(1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)", p[0], p[1], p[2], p[3], p[4], p[5]);
            snprintf(buf2, sizeof(buf2), "MoveJ(CART1,%.2f,1,1)", speed);
            call_service(buf1);
            RCLCPP_INFO(this->get_logger(), "🚀 MoveJ -> CART1, 速度: %.2f", speed);
            call_service(buf2);
        }
        else if (move_type_ == 3) { 
            double speed = input_speed > 0 ? input_speed : default_movej_speed_;
            snprintf(buf1, sizeof(buf1), "JNTPoint(1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)", p[0], p[1], p[2], p[3], p[4], p[5]);
            snprintf(buf2, sizeof(buf2), "MoveJ(JNT1,%.2f,1,1)", speed);
            call_service(buf1);
            RCLCPP_INFO(this->get_logger(), "🚀 MoveJ -> JNT1, 速度: %.2f", speed);
            call_service(buf2);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}