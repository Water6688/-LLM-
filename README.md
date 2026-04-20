# Lerbot_Control

基于 ROS 2 Humble 的 6 自由度机械臂控制工程，支持硬件反馈、RViz 数字孪生、MoveIt2 规划执行，以及远程大模型自然语言控制。

## 项目亮点

- 硬件-仿真同步：实体机械臂状态实时映射到 RViz
- MoveIt2 规划：支持标准轨迹控制器执行
- 大模型接入：远程接口返回 6 路舵机目标，自动转换为关节轨迹
- 模块化架构：描述、硬件、启动、MoveIt 配置、控制桥接分包清晰

## 大模型部署信息

- 推理 GPU：RTX 4090
- 远程模型：Qwen2.5-7B-Instruct
- 服务框架：FastAPI
- 当前 API：POST /robot_command，返回机器人动作序列

完整远程部署教程与服务代码见：[src/docs/REMOTE_MODEL_DEPLOYMENT.md](src/docs/REMOTE_MODEL_DEPLOYMENT.md)

## 系统架构

```text
用户自然语言
   |
   v
远程大模型服务 (HTTP)
   |
   v
llm_control.py 解析动作组 (0-4095)
   |
   v
FollowJointTrajectory (arm_controller + gripper_controller)
   |
   v
ros2_control 硬件接口 (STS3215)
   |
   +--> 实体机械臂执行
   |
   +--> /joint_states --> RViz / MoveIt2 可视化同步
```

## 环境要求

1. Ubuntu 22.04
2. ROS 2 Humble
3. colcon / xacro / rviz2 / ros2_control / MoveIt2
4. 串口设备（默认 `/dev/ttyUSB0`）

## 快速开始

### 1. 构建工程

```bash
cd ~/Lerbot_Control
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. 启动硬件 + MoveIt2 + RViz

```bash
source install/setup.bash
ros2 launch my_robot_bringup hardware_moveit_rviz.launch.py usb_port:=/dev/ttyUSB0
```

### 3. 启动远程大模型控制桥接

新开一个终端：

```bash
cd ~/Lerbot_Control
source install/setup.bash
python3 src/my_robot_control/llm_control.py
```

可选：自定义大模型服务地址。

```bash
export LLM_ROBOT_API_URL="https://your-server/robot_command"
python3 src/my_robot_control/llm_control.py
```

## 使用教程

### 教程 A：硬件反馈模式（只读）

用途：仅观察实体机械臂状态在 RViz 的同步，不下发控制指令。

```bash
source install/setup.bash
ros2 launch my_robot_bringup hardware_feedback_rviz.launch.py usb_port:=/dev/ttyUSB0
```

### 教程 B：硬件控制模式（可写）+ MoveIt2

用途：通过 MoveIt2 执行规划轨迹到实体机械臂。

```bash
source install/setup.bash
ros2 launch my_robot_bringup hardware_moveit_rviz.launch.py usb_port:=/dev/ttyUSB0
```

验证控制器是否在线：

```bash
source install/setup.bash
ros2 action list | grep follow_joint_trajectory
```

预期至少包含：

- `/arm_controller/follow_joint_trajectory`
- `/gripper_controller/follow_joint_trajectory`

### 教程 C：大模型自然语言控制

1. 先启动教程 B 的整套系统
2. 再运行 `llm_control.py`
3. 在交互终端输入自然语言指令
4. 脚本向远程接口请求动作组并下发轨迹
5. 如果需要独立部署服务端，请参考 [docs/REMOTE_MODEL_DEPLOYMENT.md](docs/REMOTE_MODEL_DEPLOYMENT.md)

## 远程模型返回格式约定

每组动作必须对应 6 个舵机值，范围 `0-4095`。

支持格式示例 1：

```json
{
  "robot_command": [2048, 1800, 2300, 2048, 2048, 1500]
}
```

支持格式示例 2：

```json
{
  "robot_commands": [
    [2048, 1800, 2300, 2048, 2048, 1500],
    [2100, 1700, 2200, 2100, 2000, 1700]
  ]
}
```

脚本会自动完成：

1. 多种字段名兼容提取（如 `robot_command`、`commands`、`actions`）
2. 舵机值限幅到 `0-4095`
3. 映射到关节弧度并按控制器分组下发

## 代码结构

```text
Lerbot_Control/
├── README.md
├── test.urdf
├── src/
│   ├── my_robot_control/
│   │   └── llm_control.py
│   ├── my_robot_bringup/
│   │   ├── config/
│   │   │   ├── my_robot_controllers.yaml
│   │   │   └── my_robot_feedback_only.yaml
│   │   ├── launch/
│   │   │   ├── hardware_feedback_rviz.launch.py
│   │   │   ├── hardware_moveit_rviz.launch.py
│   │   │   ├── my_robot.launch.py
│   │   │   └── my_robot_gazebo.launch.py
│   │   ├── scripts/
│   │   │   ├── angle_sync_test.py
│   │   │   ├── arm_control_test.py
│   │   │   ├── joint_state_monitor.py
│   │   │   └── joint_state_test.py
│   │   ├── worlds/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── my_robot_description/
│   │   ├── urdf/
│   │   │   ├── my_robot.urdf.xacro
│   │   │   ├── my_robot.ros2_control.xacro
│   │   │   ├── genkiarm.xacro
│   │   │   └── common_properties.xacro
│   │   ├── rviz/
│   │   ├── meshes/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── my_robot_hardware/
│   │   ├── include/
│   │   ├── scripts/
│   │   ├── src/
│   │   │   ├── my_robot_hardware.cpp
│   │   │   ├── set_arm_center.cpp
│   │   │   ├── set_position_example.cpp
│   │   │   └── sts3215_example.cpp
│   │   ├── my_robot_hardware_plugin_description.xml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── my_robot_moveit_config/
│   │   ├── config/
│   │   │   ├── my_robot.srdf
│   │   │   ├── moveit.rviz
│   │   │   ├── moveit_controllers.yaml
│   │   │   ├── ros2_controllers.yaml
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── initial_positions.yaml
│   │   │   └── pilz_cartesian_limits.yaml
│   │   ├── launch/
│   │   │   ├── move_group.launch.py
│   │   │   ├── moveit_rviz.launch.py
│   │   │   └── demo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── feetech_ros2_driver/
│       ├── feetech_driver/
│       ├── include/
│       ├── src/
│       ├── doc/
│       ├── CMakeLists.txt
│       └── package.xml
├── build/
├── install/
└── log/
```

## 常见问题

### 1. 启动时报 URDF 路径不存在

使用 `hardware_moveit_rviz.launch.py`，并确保已重新编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

### 2. 出现 `sync_read failed ... fallback per-joint read succeeded`

这是硬件读取的降级告警，表示同步读失败后已回退为逐关节读取。系统通常仍可继续运行，但建议检查串口线缆、电源和舵机通信质量。

### 3. 终端显示退出码 143

通常表示进程被外部终止（例如 Ctrl+C 或终端关闭），并不一定是程序崩溃。

## 后续规划

- 作为机械臂学习的平台
- 复现Hugging face的Lerobot开源项目
- 机器人运动控制
