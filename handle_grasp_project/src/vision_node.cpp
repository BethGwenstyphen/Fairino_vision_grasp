#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <fairino_msgs/msg/robot_nonrt_state.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <cmath>

using json = nlohmann::json;

class VisionGraspNode : public rclcpp::Node {
public:
    VisionGraspNode() : Node("vision_grasp_node") {
        // 1. 声明节点参数
        this->declare_parameter("offset_x", 0.0);
        this->declare_parameter("offset_y", 0.0);
        this->declare_parameter("offset_z", 240.0);
        this->declare_parameter("offset_rz", -40.0);
        this->declare_parameter("fixed_rx", 180.0);
        this->declare_parameter("fixed_ry", 0.0);
        this->declare_parameter("voxel_size", 0.005);
        this->declare_parameter("outlier_neighbors", 20);
        this->declare_parameter("outlier_std_ratio", 1.5);
        this->declare_parameter("marker_length", 0.05);

        // 2. 载入外参
        try {
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("handle_grasp_project");
            std::string json_path = pkg_share_dir + "/config/hand_eye_extrinsic.json";
            std::ifstream f(json_path);
            json data = json::parse(f);
            
            T_cam2flange_ = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    T_cam2flange_(i, j) = data["T_cam2flange"][i][j];
                }
            }
            RCLCPP_INFO(this->get_logger(), "✅ 成功加载手眼外参矩阵");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ 加载外参文件失败: %s", e.what());
            throw;
        }

        // 3. 初始化 ROS 通信
        auto qos = rclcpp::SensorDataQoS();
        
        sub_state_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>(
            "nonrt_state_data", 10, std::bind(&VisionGraspNode::state_callback, this, std::placeholders::_1));
        
        sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", qos, std::bind(&VisionGraspNode::pc_callback, this, std::placeholders::_1));
            
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10, std::bind(&VisionGraspNode::info_callback, this, std::placeholders::_1));
            
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&VisionGraspNode::image_callback, this, std::placeholders::_1));

        target_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/grasp_target_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/handle_center_marker", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/target_detected", 10);

        RCLCPP_INFO(this->get_logger(), "👁️ C++ 视觉节点已启动：ArUco使用纯净外参，点云把手使用带补偿外参。");
    }

private:
    Eigen::Matrix4d T_cam2flange_;
    bool aruco_detected_ = false;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    std::vector<double> current_tcp_pose_;
    bool has_tcp_ = false;

    // EMA 防抖变量
    Eigen::Vector3d smoothed_apex_ = Eigen::Vector3d::Zero();
    Eigen::Vector2d smoothed_dir_ = Eigen::Vector2d::Zero();
    bool is_first_frame_ = true;

    // =========================================================
    // 核心数学工具
    // =========================================================
    Eigen::Matrix4d euler_to_matrix(const std::vector<double>& pose) {
        double x = pose[0], y = pose[1], z = pose[2];
        double rx = pose[3] * M_PI / 180.0;
        double ry = pose[4] * M_PI / 180.0;
        double rz = pose[5] * M_PI / 180.0;

        // 对齐 Python scipy 'xyz' extrinsic: R = Rz * Ry * Rx
        Eigen::Matrix3d rot;
        rot = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = rot;
        T(0,3) = x; T(1,3) = y; T(2,3) = z;
        return T;
    }

    void state_callback(const fairino_msgs::msg::RobotNonrtState::SharedPtr msg) {
        current_tcp_pose_ = {
            msg->cart_x_cur_pos, msg->cart_y_cur_pos, msg->cart_z_cur_pos,
            msg->cart_a_cur_pos, msg->cart_b_cur_pos, msg->cart_c_cur_pos
        };
        has_tcp_ = true;
    }

    // =========================================================
    // 可视化模块
    // =========================================================
    void publish_aruco_marker(const cv::Vec3d& centroid, const std::string& frame_id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();
        marker.ns = "aruco_pose";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.pose.orientation.w = 1.0;
        
        double m_len = this->get_parameter("marker_length").as_double();
        marker.scale.x = m_len; marker.scale.y = m_len; marker.scale.z = 0.005;
        marker.color.g = 1.0; marker.color.a = 0.6;
        marker_pub_->publish(marker);
    }

    void publish_grasp_arrow(const Eigen::Vector3d& centroid, double grasp_yaw, const std::string& frame_id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();
        marker.ns = "handle_pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = centroid.x();
        marker.pose.position.y = centroid.y();
        marker.pose.position.z = centroid.z();
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = std::sin(grasp_yaw / 2.0);
        marker.pose.orientation.w = std::cos(grasp_yaw / 2.0);
        
        marker.scale.x = 0.15; marker.scale.y = 0.02; marker.scale.z = 0.02;
        marker.color.r = 1.0; marker.color.a = 1.0;
        marker_pub_->publish(marker);
    }

    // =========================================================
    // 模块 A: ArUco 纯净坐标解算通道
    // =========================================================
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!camera_matrix_.empty()) return;
        
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i) dist_coeffs_.at<double>(i) = msg->d[i];
        
        RCLCPP_INFO(this->get_logger(), "✅ 相机内参获取成功，已锁定。");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (camera_matrix_.empty()) return;
        
        cv_bridge::CvImagePtr cv_ptr;
        try { cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); }
        catch (cv_bridge::Exception& e) { return; }

        double m_len = this->get_parameter("marker_length").as_double();
        
        // OpenCV 4.x ArUco API
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, parameters);

        bool found = false;
        if (!ids.empty()) {
            std::vector<cv::Point3f> obj_points = {
                cv::Point3f(-m_len/2, m_len/2, 0), cv::Point3f(m_len/2, m_len/2, 0),
                cv::Point3f(m_len/2, -m_len/2, 0), cv::Point3f(-m_len/2, -m_len/2, 0)
            };

            for (size_t i = 0; i < ids.size(); ++i) {
                if (ids[i] == 0) {
                    found = true;
                    aruco_detected_ = true;
                    
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(obj_points, corners[i], camera_matrix_, dist_coeffs_, rvec, tvec);
                    
                    cv::Mat R_cv;
                    cv::Rodrigues(rvec, R_cv);
                    Eigen::Vector3d v_cam(R_cv.at<double>(0,0), R_cv.at<double>(1,0), R_cv.at<double>(2,0));
                    
                    publish_aruco_marker(tvec, msg->header.frame_id);
                    std_msgs::msg::Int8 status_msg; status_msg.data = 1;
                    status_pub_->publish(status_msg);
                    
                    if (has_tcp_) calculate_and_publish_aruco(tvec, v_cam);
                    break;
                }
            }
        }
        if (!found) aruco_detected_ = false;
    }

    void calculate_and_publish_aruco(const cv::Vec3d& centroid, const Eigen::Vector3d& v_cam) {
        Eigen::Vector4d P_cam(centroid[0]*1000.0, centroid[1]*1000.0, centroid[2]*1000.0, 1.0);
        Eigen::Matrix4d T_base2flange = euler_to_matrix(current_tcp_pose_);
        Eigen::Vector4d P_base = T_base2flange * T_cam2flange_ * P_cam;
        
        Eigen::Vector3d vec_base = (T_base2flange.block<3,3>(0,0) * T_cam2flange_.block<3,3>(0,0)) * v_cam;
        double raw_rz = std::atan2(vec_base.y(), vec_base.x()) * 180.0 / M_PI;
        
        double home_rz = current_tcp_pose_[5];
        double diff = std::fmod(raw_rz - home_rz + 180.0, 360.0) - 180.0;
        if (diff > 90.0) diff -= 180.0;
        else if (diff < -90.0) diff += 180.0;
        
        double final_rz = home_rz + diff;
        while (final_rz > 180.0) final_rz -= 360.0;
        while (final_rz < -180.0) final_rz += 360.0;
        
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {P_base.x(), P_base.y(), P_base.z(), 
                    this->get_parameter("fixed_rx").as_double(), 
                    this->get_parameter("fixed_ry").as_double(), final_rz};
        target_pose_pub_->publish(msg);
    }

    // =========================================================
    // 模块 B: 点云把手补偿通道
    // =========================================================
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (aruco_detected_) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // 1. 体素降采样与 ROI 裁剪
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(this->get_parameter("voxel_size").as_double(), 
                       this->get_parameter("voxel_size").as_double(), 
                       this->get_parameter("voxel_size").as_double());
        vg.filter(*cloud_filtered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ> box;
        box.setMin(Eigen::Vector4f(-0.5, -0.5, 0.0, 1.0));
        box.setMax(Eigen::Vector4f(0.5, 0.5, 0.8, 1.0));
        box.setInputCloud(cloud_filtered);
        box.filter(*cloud_cropped);

        if (cloud_cropped->points.size() < 100) { publish_status(0); return; }

        // 2. RANSAC 提取平面
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.015);
        seg.setInputCloud(cloud_cropped);
        seg.segment(*inliers, *coeffs);

        if (inliers->indices.size() < 100) { publish_status(0); return; }

        float A = coeffs->values[0], B = coeffs->values[1], C = coeffs->values[2], D = coeffs->values[3];
        if (C < 0) { A = -A; B = -B; C = -C; D = -D; }
        Eigen::Vector3f normal_vec(A, B, C);
        float normal_norm = normal_vec.norm();

        // 3. 点到平面距离过滤
        pcl::PointIndices::Ptr handle_idx(new pcl::PointIndices());
        for (size_t i = 0; i < cloud_cropped->points.size(); ++i) {
            Eigen::Vector3f pt(cloud_cropped->points[i].x, cloud_cropped->points[i].y, cloud_cropped->points[i].z);
            float dist = (pt.dot(normal_vec) + D) / normal_norm;
            if (dist < -0.03 && dist > -0.08) handle_idx->indices.push_back(i);
        }

        if (handle_idx->indices.size() < 30) { publish_status(0); return; }

        pcl::PointCloud<pcl::PointXYZ>::Ptr handle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_cropped);
        extract.setIndices(handle_idx);
        extract.setNegative(false);
        extract.filter(*handle_cloud);

        // 4. DBScan 欧式聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(handle_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(15);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(handle_cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) { publish_status(0); return; }

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_handle(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& idx : cluster_indices[0].indices) raw_handle->points.push_back(handle_cloud->points[idx]);

        // 5. SOR 统计学去飞点
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_handle(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(raw_handle);
        sor.setMeanK(this->get_parameter("outlier_neighbors").as_int());
        sor.setStddevMulThresh(this->get_parameter("outlier_std_ratio").as_double());
        sor.filter(*final_handle);

        if (final_handle->points.size() < 15) { publish_status(0); return; }

        // 6. PCA 物理顶点重定向
        Eigen::Matrix<double, Eigen::Dynamic, 2> xy_points(final_handle->points.size(), 2);
        for (size_t i = 0; i < final_handle->points.size(); ++i) {
            xy_points(i, 0) = final_handle->points[i].x;
            xy_points(i, 1) = final_handle->points[i].y;
        }

        Eigen::RowVector2d centroid_2d = xy_points.colwise().mean();
        Eigen::Matrix<double, Eigen::Dynamic, 2> centered = xy_points.rowwise() - centroid_2d;
        Eigen::Matrix2d cov = (centered.adjoint() * centered) / double(xy_points.rows() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
        Eigen::Vector2d minor_axis = eig.eigenvectors().col(0); 
        Eigen::VectorXd projections = centered * minor_axis;
        
        int max_idx, min_idx;
        double max_val = projections.maxCoeff(&max_idx);
        double min_val = projections.minCoeff(&min_idx);

        int apex_idx = 0;
        Eigen::Vector2d current_dir;
        if (std::abs(max_val) > std::abs(min_val)) {
            apex_idx = max_idx; current_dir = -minor_axis; 
        } else {
            apex_idx = min_idx; current_dir = minor_axis;
        }

        Eigen::Vector3d current_apex(final_handle->points[apex_idx].x, 
                                     final_handle->points[apex_idx].y, 
                                     final_handle->points[apex_idx].z);

        // 7. EMA 防抖
        if (is_first_frame_) {
            smoothed_apex_ = current_apex;
            smoothed_dir_ = current_dir;
            is_first_frame_ = false;
        } else {
            if (current_dir.dot(smoothed_dir_) < 0) current_dir = -current_dir;
            double alpha_pos = 0.15, alpha_dir = 0.10; 
            smoothed_apex_ = alpha_pos * current_apex + (1.0 - alpha_pos) * smoothed_apex_;
            smoothed_dir_ = alpha_dir * current_dir + (1.0 - alpha_dir) * smoothed_dir_;
            smoothed_dir_.normalize(); 
        }

        double final_yaw = std::atan2(smoothed_dir_.y(), smoothed_dir_.x());

        // 8. 状态发布
        publish_status(2);
        publish_grasp_arrow(smoothed_apex_, final_yaw, msg->header.frame_id);

        if (has_tcp_) calculate_and_publish_target(smoothed_apex_, final_yaw);
    }

    void calculate_and_publish_target(const Eigen::Vector3d& centroid, double grasp_yaw_cam) {
        Eigen::Vector4d P_cam(centroid.x()*1000.0, centroid.y()*1000.0, centroid.z()*1000.0, 1.0);
        Eigen::Matrix4d T_base2flange = euler_to_matrix(current_tcp_pose_);
        Eigen::Vector4d P_base = T_base2flange * T_cam2flange_ * P_cam;
        
        double target_x = P_base.x() + this->get_parameter("offset_x").as_double();
        double target_y = P_base.y() + this->get_parameter("offset_y").as_double();
        double target_z = P_base.z() + this->get_parameter("offset_z").as_double();
        
        Eigen::Vector3d vec_cam(std::cos(grasp_yaw_cam), std::sin(grasp_yaw_cam), 0.0);
        Eigen::Vector3d vec_base = (T_base2flange.block<3,3>(0,0) * T_cam2flange_.block<3,3>(0,0)) * vec_cam;
        
        double raw_rz = std::atan2(vec_base.y(), vec_base.x()) * 180.0 / M_PI + this->get_parameter("offset_rz").as_double();
        double home_rz = current_tcp_pose_[5];
        
        double diff = std::fmod(raw_rz - home_rz + 180.0, 360.0) - 180.0;
        if (diff > 90.0) diff -= 180.0;
        else if (diff < -90.0) diff += 180.0;
            
        double final_rz = home_rz + diff;
        while (final_rz > 180.0) final_rz -= 360.0;
        while (final_rz < -180.0) final_rz += 360.0;
            
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {target_x, target_y, target_z, 
                    this->get_parameter("fixed_rx").as_double(), 
                    this->get_parameter("fixed_ry").as_double(), final_rz};
        target_pose_pub_->publish(msg);
    }

    void publish_status(int status) {
        std_msgs::msg::Int8 msg;
        msg.data = status;
        status_pub_->publish(msg);
    }

    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr sub_state_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionGraspNode>());
    rclcpp::shutdown();
    return 0;
}
