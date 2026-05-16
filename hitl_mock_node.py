import rclpy
from rclpy.node import Node
from fairino_msgs.msg import RobotNonrtState
from std_msgs.msg import Float64MultiArray, Int8
import math

class DigitalTwinSim(Node):
    def __init__(self):
        super().__init__('digital_twin_sim')
        
        # 1. 核心状态维护
        self.motion_done = 1
        self.is_moving = False
        
        # 初始状态：大致对齐到你的 MoveToObserve 点位
        self.cur_joints = [-123.093, -54.4, 69.633, -105.855, -91.203, 47.568]
        # 笛卡尔坐标初始值（假设为观察点的末端位姿）
        self.cur_cart = [300.0, 0.0, 500.0, 180.0, 0.0, 0.0] 
        
        self.target_pose = []
        self.move_type = 1 # 1/2: 笛卡尔, 3: 关节
        
        # 2. ROS 通信定义
        self.state_pub = self.create_publisher(RobotNonrtState, 'nonrt_state_data', 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/arm_control_cmd', self.cmd_callback, 10)
        self.gripper_sub = self.create_subscription(Int8, '/gripper_control_cmd', self.gripper_callback, 10)
        
        # 3. 20Hz 物理引擎时钟
        self.timer = self.create_timer(0.05, self.physics_loop)
        
        self.get_logger().info("🌐 数字孪生黑盒已启动！接管所有机械臂底层逻辑。")

    def gripper_callback(self, msg):
        state = "张开" if msg.data == 1 else "闭合"
        self.get_logger().info(f"🗜️ 物理引擎模拟: 夹爪正在【{state}】...")

    def cmd_callback(self, msg):
        if len(msg.data) < 9: return
        self.move_type = int(msg.data[0])
        # 提取目标位姿数据
        self.target_pose = list(msg.data[2:8])
        
        self.is_moving = True
        self.motion_done = 0
        self.get_logger().info(f"🚀 物理引擎截获指令: 模式 {self.move_type}, 目标 {self.target_pose[:3]}...")

    def physics_loop(self):
        # 如果正在移动，执行线性插值逼近
        if self.is_moving:
            reached = True
            step_size = 2.0 # 模拟移动步长 (速度)
            
            current_arr = self.cur_joints if self.move_type == 3 else self.cur_cart
            
            for i in range(6):
                diff = self.target_pose[i] - current_arr[i]
                if abs(diff) > step_size:
                    current_arr[i] += math.copysign(step_size, diff)
                    reached = False
                else:
                    current_arr[i] = self.target_pose[i]
                    
            if reached:
                self.is_moving = False
                self.motion_done = 1
                self.get_logger().info("✅ 物理引擎模拟: 到达目标位置！")

        # 无论是否移动，高频全网广播当前状态
        state_msg = RobotNonrtState()
        state_msg.robot_motion_done = self.motion_done
        
        state_msg.j1_cur_pos, state_msg.j2_cur_pos, state_msg.j3_cur_pos, \
        state_msg.j4_cur_pos, state_msg.j5_cur_pos, state_msg.j6_cur_pos = self.cur_joints
        
        state_msg.cart_x_cur_pos, state_msg.cart_y_cur_pos, state_msg.cart_z_cur_pos, \
        state_msg.cart_a_cur_pos, state_msg.cart_b_cur_pos, state_msg.cart_c_cur_pos = self.cur_cart
        
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()