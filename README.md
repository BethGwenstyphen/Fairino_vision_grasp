# Fairino 协作机器人视觉抓取与装配系统 (V1 稳定版)

本项目是一个基于 **ROS2 Humble** 的工业级视觉抓取流水线架构，系统集成了**无感视觉解耦**、**BehaviorTree.CPP 状态机大脑**。

---

## 1. 核心环境准备 (Essential Background)

在拉取代码前，请确保您的系统为 **Ubuntu 22.04** 并已完整安装 **ROS2 Humble**。

### 1.1 系统级依赖库

打开终端，执行以下命令安装必要的系统工具与 ROS2 桥接组件：

```bash
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-rosdep \
    iputils-ping \
    net-tools \
    libgl1-mesa-glx \
    libgomp1 \
    libusb-1.0-0 \
    python3-colcon-common-extensions \
    ros-humble-camera-info-manager \
    ros-humble-image-publisher \
    ros-humble-diagnostic-updater \
    ros-humble-rosbridge-suite \
    ros-humble-rmw-cyclonedds-cpp
```

### 1.2 Python 算法算法库

为了保证 3D 点云降维与视觉矩阵解算的绝对稳定，请严格锁定以下 Python 库版本（特别是 Numpy 版本）：

```bash
pip3 install -i [https://pypi.tuna.tsinghua.edu.cn/simple](https://pypi.tuna.tsinghua.edu.cn/simple) \
    "numpy<2.0" \
    opencv-python \
    open3d \
    setuptools==58.2.0
```

### 1.3 行为树大脑依赖

本项目的大脑中枢基于高阶的 BehaviorTree.CPP V3 构建，请执行：

```bash
sudo apt-get install ros-humble-behaviortree-cpp-v3
```

---

## 2. 工作空间构建与编译

请严格按照以下 Step-by-step 顺序执行，避免破坏底层的依赖链条。

### 2.1 创建工作空间并拉取代码

```bash
# 创建工作空间
mkdir -p ~/frc_ws/src
cd ~/frc_ws/src

# 拉取本项目主干代码
git clone [https://github.com/BethGwenstyphen/Fairino_vision_grasp.git](https://github.com/BethGwenstyphen/Fairino_vision_grasp.git).

# 独立拉取图漾相机 ROS2 驱动
git clone [https://gitee.com/percipioxyz/camport_ros2.git](https://gitee.com/percipioxyz/camport_ros2.git)
```

### 2.2 解决 Fairino 底层依赖

使用 `rosdepc` 自动补全机械臂底层的缺失项：

```bash
cd ~/frc_ws
sudo pip3 install -i [https://pypi.tuna.tsinghua.edu.cn/simple](https://pypi.tuna.tsinghua.edu.cn/simple) rosdepc
sudo rosdepc init
rosdepc update
rosdepc install -i --from-path src --rosdistro humble -y
```

### 2.3 严格按顺序编译 (Colcon Build)

请依次复制以下命令进行模块化编译：

**第一步：编译机械臂基础通信包**

```bash
colcon build --packages-select fairino_msgs
colcon build --packages-select fairino_description
colcon build --packages-select fairino_hardware
```

**第二步：编译相机驱动**

```bash
colcon build --packages-select percipio_camera --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**第三步：编译本项目的核心业务大脑**

```bash
colcon build --packages-select handle_grasp_project bt_grasp_brain
```

编译完成后，刷新环境变量：

```bash
source install/setup.bash
```

---

*注：系统内置了极其严苛的 6 秒 EMA（指数移动平均）视觉防抖机制，机械臂到达观测点后会死等 6 秒以确保坐标绝对收敛，请勿认为是系统卡顿。*