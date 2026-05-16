cmake_minimum_required(VERSION 3.8)
project(handle_grasp_project)

# 强行开启 C++17 标准与 O3 工业级性能优化
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# =========================================================
# 1. 查找依赖包 (DDS 通信与感知算法库)
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features search segmentation)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# =========================================================
# 2. 注册控制节点：control_node (V4 独立线程队列版)
# =========================================================
add_executable(control_node src/control_node.cpp)

target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 强硬绑定控制肌肉所需的 ROS2 核心依赖件
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  fairino_msgs
)

# =========================================================
# 3. 注册视觉节点：vision_node (V2 C++ 点云加速版)
# =========================================================
add_executable(vision_node src/vision_node.cpp)

target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  fairino_msgs
  cv_bridge
  OpenCV
  pcl_conversions
)

target_link_libraries(vision_node ${PCL_LIBRARIES})

# =========================================================
# 4. 安装与打包 (生成编译产物到 install 目录)
# =========================================================
# 安装二进制可执行文件
install(TARGETS control_node vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装静态参数配置文件 (params.yaml / hand_eye_extrinsic.json)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()