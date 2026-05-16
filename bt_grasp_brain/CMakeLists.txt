cmake_minimum_required(VERSION 3.8)
project(bt_grasp_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找核心依赖
# 1. 寻找核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)  
find_package(behaviortree_cpp_v3 REQUIRED)

# 2. 编译你的大脑主程序
add_executable(bt_main_node src/main_tree_node.cpp)

# 3. 链接头文件与依赖库
target_include_directories(bt_main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bt_main_node
  rclcpp
  std_msgs
  fairino_msgs
  behaviortree_cpp_v3
  ament_index_cpp
)

# 4. 强制安装规则 (Critical)
install(TARGETS bt_main_node
  DESTINATION lib/${PROJECT_NAME})

# 【关键点】让 ROS 2 安装你的 XML 行为树图纸，否则运行时找不到！
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees)

ament_package()