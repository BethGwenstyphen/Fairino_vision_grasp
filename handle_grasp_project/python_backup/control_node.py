import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8
from fairino_msgs.msg import RobotNonrtState
from fairino_msgs.srv import RemoteCmdInterface

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # 1. 建立服务客户端
        self.client = self.create_client(RemoteCmdInterface, '/fairino_remote_command_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ 等待机器人命令服务启动...')

        # 2. 状态监控变量
        self.motion_done = True
        self.is_executing = False
        self.cmd_sent_time = 0.0
        self.current_gripper_state = -1 # -1: 未知, 0: 闭合, 1: 张开

        # 3. 默认速度设置
        self.default_movel_speed = 15.0
        self.default_movej_speed = 10.0

        # 4. 订阅与发布
        self.state_sub = self.create_subscription(RobotNonrtState, 'nonrt_state_data', self.state_callback, 10)
        
        # 机械臂控制订阅
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/arm_control_cmd', self.control_callback, 10)
        
        # 夹爪控制订阅与状态发布
        self.gripper_sub = self.create_subscription(Int8, '/gripper_control_cmd', self.gripper_callback, 10)
        self.gripper_pub = self.create_publisher(Int8, '/gripper_status', 10)

        self.init_robot()
        self.get_logger().info("✅ 控制节点 V3 已就绪：机械臂运动与 I/O 夹爪控制已整合。")

    def call_service(self, cmd_str):
        clean_cmd = cmd_str.replace(" ", "")
        req = RemoteCmdInterface.Request()
        req.cmd_str = clean_cmd
        return self.client.call_async(req)

    def init_robot(self):
        self.get_logger().info("🤖 初始化硬件状态...")
        self.call_service('ResetAllError()')
        self.call_service('RobotEnable(1)')
        self.call_service('Mode(0)')      
        self.call_service('SetSpeed(20)') 
        self.call_service('SetToolCoord(1,0,0,0,0,0,0)')
        self.call_service('SetWObjCoord(1,0,0,0,0,0,0)')
        
        # 默认将夹爪置为张开状态，确保安全
        self.get_logger().info("🗜️ 默认初始化夹爪为张开状态...")
        self.call_service('SetToolDO(0,0)')
        self.call_service('SetToolDO(1,1)')
        self.current_gripper_state = 1
        self.gripper_pub.publish(Int8(data=1))

    def state_callback(self, msg):
        self.motion_done = bool(msg.robot_motion_done)
        if self.is_executing:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.cmd_sent_time
            if elapsed > 0.5 and self.motion_done:
                self.get_logger().info(f"🎯 到达目标！当前 Z: {msg.cart_z_cur_pos:.1f}")
                self.is_executing = False

    def gripper_callback(self, msg):
        """处理夹爪 I/O 控制指令"""
        cmd = msg.data
        if cmd == 1:
            self.get_logger().info("🗜️ 收到指令：张开夹爪")
            self.call_service('SetToolDO(0,0)')  # 撤销闭合力
            self.call_service('SetToolDO(1,1)')  # 施加张开力
            self.current_gripper_state = 1
            self.gripper_pub.publish(Int8(data=1)) # 反馈状态
        elif cmd == 0:
            self.get_logger().info("🗜️ 收到指令：闭合夹爪")
            self.call_service('SetToolDO(1,0)')  # 撤销张开力
            self.call_service('SetToolDO(0,1)')  # 施加闭合力
            self.current_gripper_state = 0
            self.gripper_pub.publish(Int8(data=0)) # 反馈状态
        else:
            self.get_logger().warn(f"⚠️ 未知的夹爪指令: {cmd}。仅支持 1(张开) 或 0(闭合)。")

    def control_callback(self, msg):
        """处理机械臂运动指令"""
        if self.is_executing:
            self.get_logger().warn("✋ 机器人忙碌中，忽略指令。")
            return

        if len(msg.data) < 8:
            self.get_logger().error("❌ 数据长度不足，协议需 8 位: [类型, 速度, X, Y, Z, Rx, Ry, Rz]")
            return

        move_type = int(msg.data[0])
        input_speed = msg.data[1]
        p = msg.data[2:8] 

        if move_type == 1: 
            speed = input_speed if input_speed > 0 else self.default_movel_speed
            self.call_service(f"CARTPoint(1,{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},{p[3]:.3f},{p[4]:.3f},{p[5]:.3f})")
            self.get_logger().info(f"🚀 MoveL -> CART1, 速度: {speed}")
            self.call_service(f"MoveL(CART1,{speed},1,1)")

        elif move_type == 2: 
            speed = input_speed if input_speed > 0 else self.default_movej_speed
            self.call_service(f"CARTPoint(1,{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},{p[3]:.3f},{p[4]:.3f},{p[5]:.3f})")
            self.get_logger().info(f"🚀 MoveJ -> CART1, 速度: {speed}")
            self.call_service(f"MoveJ(CART1,{speed},1,1)")

        elif move_type == 3: 
            speed = input_speed if input_speed > 0 else self.default_movej_speed
            self.call_service(f"JNTPoint(1,{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},{p[3]:.3f},{p[4]:.3f},{p[5]:.3f})")
            self.get_logger().info(f"🚀 MoveJ -> JNT1, 速度: {speed}")
            self.call_service(f"MoveJ(JNT1,{speed},1,1)")

        self.is_executing = True
        self.cmd_sent_time = self.get_clock().now().nanoseconds / 1e9

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()