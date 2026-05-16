import rclpy
from rclpy.node import Node
from fairino_msgs.msg import RobotNonrtState

class JointWatcher(Node):
    def __init__(self):
        super().__init__('joint_watcher')
        # 强制硬连线订阅话题
        self.sub = self.create_subscription(RobotNonrtState, 'nonrt_state_data', self.cb, 10)
        self.get_logger().info("🔍 正在强制抓取关节实时数据...")

    def cb(self, msg):
        # 格式化输出，方便你直接复制
        joints = [
            round(msg.j1_cur_pos, 3), round(msg.j2_cur_pos, 3), round(msg.j3_cur_pos, 3),
            round(msg.j4_cur_pos, 3), round(msg.j5_cur_pos, 3), round(msg.j6_cur_pos, 3)
        ]
        print(f"\n✅ 当前 6 轴关节坐标 (Joints):")
        print(f"target_pos_ = {{{joints[0]}, {joints[1]}, {joints[2]}, {joints[3]}, {joints[4]}, {joints[5]}}};")

def main():
    rclpy.init()
    node = JointWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()