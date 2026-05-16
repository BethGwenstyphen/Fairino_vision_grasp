import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray, Int8
from fairino_msgs.msg import RobotNonrtState

from cv_bridge import CvBridge
import cv2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data 
from ament_index_python.packages import get_package_share_directory

import open3d as o3d
import numpy as np
import json
import os
import collections
from scipy.spatial.transform import Rotation as R

class VisionGraspNode(Node):
    def __init__(self):
        super().__init__('vision_grasp_node')
        
        # 1. 声明节点参数 (把手专属补偿)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('offset_x', 0.0),
                ('offset_y', 0.0),
                ('offset_z', 240.0),
                ('offset_rz', -40.0),
                ('fixed_rx', 180.0),  
                ('fixed_ry', 0.0),
                ('voxel_size', 0.005),
                ('outlier_neighbors', 20),
                ('outlier_std_ratio', 1.5),
                ('marker_length', 0.05)
            ]
        )

        # 2. 载入外参
        try:
            pkg_share_dir = get_package_share_directory('handle_grasp_project')
            json_path = os.path.join(pkg_share_dir, 'config', 'hand_eye_extrinsic.json')
            with open(json_path, 'r') as f:
                self.extrinsic_data = json.load(f)
            self.T_cam2flange = np.array(self.extrinsic_data["T_cam2flange"])
            self.get_logger().info("✅ 成功加载手眼外参矩阵")
        except Exception as e:
            self.get_logger().error(f"❌ 加载外参文件失败: {e}")
            raise e

        # 3. 初始化 ROS 通信
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_detected = False

        self.sub_state = self.create_subscription(RobotNonrtState, 'nonrt_state_data', self.state_callback, 10)
        self.sub_pc = self.create_subscription(PointCloud2, '/camera/depth_registered/points', self.pc_callback, qos_profile_sensor_data)
        
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)
        self.img_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        
        self.target_pose_pub = self.create_publisher(Float64MultiArray, '/grasp_target_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/handle_center_marker', 10)
        self.status_pub = self.create_publisher(Int8, '/target_detected', 10)
        
        self.current_tcp_pose = None
        
        # EMA 时序防抖状态变量（替代旧的 history 队列）
        self.smoothed_apex = None
        self.smoothed_dir = None

        self.get_logger().info("👁️ 视觉节点已启动：ArUco使用纯净外参，点云把手使用带补偿外参。")

    def euler_to_matrix(self, pose):
        x, y, z, rx, ry, rz = pose
        rot = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[0, 3], T[1, 3], T[2, 3] = x, y, z
        return T

    def state_callback(self, msg):
        self.current_tcp_pose = [
            msg.cart_x_cur_pos, msg.cart_y_cur_pos, msg.cart_z_cur_pos,
            msg.cart_a_cur_pos, msg.cart_b_cur_pos, msg.cart_c_cur_pos
        ]

    # =========================================================
    # 可视化模块
    # =========================================================
    def publish_aruco_marker(self, centroid, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aruco_pose"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(centroid[0])
        marker.pose.position.y = float(centroid[1])
        marker.pose.position.z = float(centroid[2])
        marker.pose.orientation.w = 1.0
        
        m_len = self.get_parameter('marker_length').value
        marker.scale.x = float(m_len)
        marker.scale.y = float(m_len)
        marker.scale.z = 0.005
        marker.color.g = 1.0
        marker.color.a = 0.6
        self.marker_pub.publish(marker)

    def publish_grasp_arrow(self, centroid, grasp_yaw, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "handle_pose"
        marker.id = 0
        marker.type = Marker.ARROW 
        marker.action = Marker.ADD
        marker.pose.position.x = float(centroid[0])
        marker.pose.position.y = float(centroid[1])
        marker.pose.position.z = float(centroid[2])
        
        q_w = float(np.cos(grasp_yaw / 2.0))
        q_z = float(np.sin(grasp_yaw / 2.0))
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = q_z
        marker.pose.orientation.w = q_w
        
        marker.scale.x = 0.15
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    # =========================================================
    # 模块 A: ArUco 纯净坐标解算通道
    # =========================================================
    def info_callback(self, msg):
        # 状态拦截：如果已经接收过参数，直接丢弃后续消息，保护底层 WaitSet
        if self.camera_matrix is not None:
            return
            
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("✅ 相机内参获取成功，已锁定。")
        # 绝对禁止在此处调用 self.info_sub.destroy()

    def image_callback(self, msg):
        # 1. 架构解耦：仅依赖相机内参，绝不依赖机械臂状态
        if self.camera_matrix is None: 
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        m_len = self.get_parameter('marker_length').value
        
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(cv_image)
        except AttributeError:
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
        
        found = False
        if ids is not None:
            obj_points = np.array([[-m_len/2, m_len/2, 0], [m_len/2, m_len/2, 0], [m_len/2, -m_len/2, 0], [-m_len/2, -m_len/2, 0]], dtype=np.float32)
            for i in range(len(ids)):
                if ids[i][0] == 0:
                    found = True
                    self.aruco_detected = True
                    
                    _, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs)
                    centroid = tvec.flatten()
                    v_cam = cv2.Rodrigues(rvec)[0][:, 0]
                    
                    # 2. 纯视觉管线：立刻渲染 RViz 并发布识别状态 (不依赖机械臂)
                    self.publish_aruco_marker(centroid, msg.header.frame_id)
                    msg_status = Int8()
                    msg_status.data = 1
                    self.status_pub.publish(msg_status)
                    
                    # 3. 运动学管线拦截：只有在机械臂在线时，才进行坐标映射与目标发布
                    if self.current_tcp_pose is not None:
                        self.calculate_and_publish_aruco(centroid, v_cam)
                    break
                    
        if not found:
            self.aruco_detected = False

    def calculate_and_publish_aruco(self, centroid, v_cam):
        """【ArUco 专线】纯净解算，绝对剔除 offset_z 及其他任何补偿"""
        home_x, home_y, home_z, home_rx, home_ry, home_rz = self.current_tcp_pose
        
        # 将米的 centroid 转换为毫米，直接上外参
        P_cam = np.array([[centroid[0]*1000.0], [centroid[1]*1000.0], [centroid[2]*1000.0], [1.0]])
        T_base2flange = self.euler_to_matrix(self.current_tcp_pose)
        P_base = T_base2flange @ self.T_cam2flange @ P_cam
        
        # 绝不加偏移量，直接读取原始值
        target_x = P_base[0, 0]
        target_y = P_base[1, 0]
        target_z = P_base[2, 0]
        
        # 姿态转换
        vec_base = (T_base2flange[:3, :3] @ self.T_cam2flange[:3, :3]) @ v_cam
        raw_rz = np.degrees(np.arctan2(vec_base[1], vec_base[0]))
        
        diff = (raw_rz - home_rz + 180.0) % 360.0 - 180.0
        if diff > 90.0: diff -= 180.0
        elif diff < -90.0: diff += 180.0
        final_rz = home_rz + diff
        while final_rz > 180.0: final_rz -= 360.0
        while final_rz < -180.0: final_rz += 360.0
        
        f_rx = self.get_parameter('fixed_rx').value
        f_ry = self.get_parameter('fixed_ry').value
        
        target_msg = Float64MultiArray()
        target_msg.data = [target_x, target_y, target_z, f_rx, f_ry, final_rz]
        self.target_pose_pub.publish(target_msg)

    # =========================================================
    # 模块 B: 点云把手补偿通道 (严格保持原逻辑不变)
    # =========================================================
    def pc_callback(self, msg):
        # 1. 架构解耦：移除对 self.current_tcp_pose 的前置依赖
        if self.aruco_detected: return

        v_size = self.get_parameter('voxel_size').value
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # 强制解包：打破 C++ 结构化内存块，显式提取 x, y, z 重组为纯净的 Nx3 矩阵
        points_list = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float64)
        
        if len(points_list) < 100: 
            self.status_pub.publish(Int8(data=0))
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_list)
        pcd = pcd.voxel_down_sample(voxel_size=v_size)

        # 保留原有的 ROI 裁剪限制
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.5, -0.5, 0.0), max_bound=(0.5, 0.5, 0.8))
        pcd_cropped = pcd.crop(bbox)
        if len(pcd_cropped.points) < 100: 
            self.status_pub.publish(Int8(data=0))
            return

        # 2. RANSAC 提取基准平面 (替代原有的粗暴多层切片)
        plane_model, inliers = pcd_cropped.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=1000)
        if len(inliers) < 100: 
            self.status_pub.publish(Int8(data=0))
            return

        A, B, C, D = plane_model
        if C < 0: A, B, C, D = -A, -B, -C, -D # 统一法向朝向相机

        normal_vec = np.array([A, B, C])
        normal_norm = np.linalg.norm(normal_vec)

        # 3. 点到平面距离过滤 (免疫相机倾斜与平面起伏)
        all_points = np.asarray(pcd_cropped.points)
        distances = (np.dot(all_points, normal_vec) + D) / normal_norm
        handle_indices = np.where((distances < -0.03) & (distances > -0.08))[0]

        if len(handle_indices) < 30: 
            self.status_pub.publish(Int8(data=0))
            return
            
        handle_pcd = pcd_cropped.select_by_index(handle_indices)

        # 4. DBScan 欧式聚类提取最大块
        labels = np.array(handle_pcd.cluster_dbscan(eps=0.02, min_points=15))
        if labels.max() < 0: 
            self.status_pub.publish(Int8(data=0))
            return 
            
        largest_cluster_idx = np.argmax(np.bincount(labels[labels >= 0]))
        raw_handle_points = np.asarray(handle_pcd.points)[labels == largest_cluster_idx]

        # 5. 统计学去飞点毛刺 (对标 C++ 的 SOR)
        cluster_pcd = o3d.geometry.PointCloud()
        cluster_pcd.points = o3d.utility.Vector3dVector(raw_handle_points)
        nb_neighbors = self.get_parameter('outlier_neighbors').value
        std_ratio = self.get_parameter('outlier_std_ratio').value
        final_pcd, _ = cluster_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        final_handle_points = np.asarray(final_pcd.points)

        if len(final_handle_points) < 15: 
            self.status_pub.publish(Int8(data=0))
            return

        # 6. PCA 降维与物理顶点 (Apex) 锁定
        xy_points = final_handle_points[:, :2]
        centroid_2d = np.mean(xy_points, axis=0)
        centered_points = xy_points - centroid_2d
        
        cov_matrix = np.cov(centered_points, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        minor_axis = eigenvectors[:, 0] # eigh 默认升序，第一列为次轴
        
        projections = np.dot(centered_points, minor_axis)
        max_idx = np.argmax(projections)
        min_idx = np.argmin(projections)
        
        # 强制方向指向 U 型内部
        if abs(projections[max_idx]) > abs(projections[min_idx]):
            apex_idx = max_idx
            current_dir = -minor_axis
        else:
            apex_idx = min_idx
            current_dir = minor_axis

        current_apex = final_handle_points[apex_idx]

        # 7. EMA 时序锁定与防抖
        if self.smoothed_apex is None:
            self.smoothed_apex = current_apex
            self.smoothed_dir = current_dir
        else:
            # 暴力制止 180 度翻转
            if np.dot(current_dir, self.smoothed_dir) < 0:
                current_dir = -current_dir
                
            alpha_pos = 0.15 
            alpha_dir = 0.10 
            self.smoothed_apex = alpha_pos * current_apex + (1.0 - alpha_pos) * self.smoothed_apex
            self.smoothed_dir = alpha_dir * current_dir + (1.0 - alpha_dir) * self.smoothed_dir
            self.smoothed_dir /= np.linalg.norm(self.smoothed_dir)

        final_yaw = np.arctan2(self.smoothed_dir[1], self.smoothed_dir[0])

        # 8. 发布状态与可视化
        self.status_pub.publish(Int8(data=2))
        self.publish_grasp_arrow(self.smoothed_apex, final_yaw, msg.header.frame_id)

        # 9. 仅在最后一步与机器人产生耦合
        if self.current_tcp_pose is not None:
            self.calculate_and_publish_target(self.smoothed_apex, final_yaw)

    def calculate_and_publish_target(self, centroid, grasp_yaw_cam):
        """【把手专线】带悬停高度和偏置的控制解算"""
        off_x = self.get_parameter('offset_x').value
        off_y = self.get_parameter('offset_y').value
        off_z = self.get_parameter('offset_z').value
        off_rz = self.get_parameter('offset_rz').value
        f_rx = self.get_parameter('fixed_rx').value
        f_ry = self.get_parameter('fixed_ry').value

        home_x, home_y, home_z, home_rx, home_ry, home_rz = self.current_tcp_pose
        
        P_cam = np.array([[centroid[0]*1000.0], [centroid[1]*1000.0], [centroid[2]*1000.0], [1.0]])
        T_base2flange = self.euler_to_matrix(self.current_tcp_pose)
        P_base = T_base2flange @ self.T_cam2flange @ P_cam
        
        # 点云把手专用：叠加 offset 参数
        target_x = P_base[0, 0] + off_x
        target_y = P_base[1, 0] + off_y
        target_z = P_base[2, 0] + off_z 
        
        vec_cam = np.array([np.cos(grasp_yaw_cam), np.sin(grasp_yaw_cam), 0.0])
        R_base2cam = T_base2flange[:3, :3] @ self.T_cam2flange[:3, :3]
        vec_base = R_base2cam @ vec_cam
        
        raw_target_rz = np.degrees(np.arctan2(vec_base[1], vec_base[0])) + off_rz
        
        diff = (raw_target_rz - home_rz + 180.0) % 360.0 - 180.0
        if diff > 90.0: diff -= 180.0
        elif diff < -90.0: diff += 180.0
            
        final_rz = home_rz + diff
        while final_rz > 180.0: final_rz -= 360.0
        while final_rz < -180.0: final_rz += 360.0
            
        target_msg = Float64MultiArray()
        target_msg.data = [target_x, target_y, target_z, f_rx, f_ry, final_rz]
        self.target_pose_pub.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()