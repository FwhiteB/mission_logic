# Pipeline Mission Logic

这个工作区用于让机器狗执行管线探测任务：机器狗根据自身定位和接收器读数，自动规划搜索、居中、测量、沿管线前进以及丢线后的重捕获路线。

核心目标不是控制底层运动学，而是输出机器狗可以执行的导航目标点 `/goal_pose`。机器狗需要提供定位 `/state_estimation`，并能够执行 `/goal_pose`。

## 目录结构

```text
.
├── mission_logic/
│   ├── config/
│   │   ├── mission_logic.yaml
│   │   └── straight_wire.json
│   ├── launch/
│   │   ├── mission_logic.launch.py
│   │   ├── demo_sim_signal.launch.py
│   │   └── demo_real_signal.launch.py
│   ├── mission_logic/
│   │   ├── mission_node.py
│   │   ├── magnetic_field_node.py
│   │   ├── models.py
│   │   ├── geometry.py
│   │   └── old_field_sim.py
│   ├── package.xml
│   └── setup.py
└── mission_logic_msgs/
    ├── msg/
    │   └── SensorMsg.msg
    ├── CMakeLists.txt
    └── package.xml
```

## 功能概览

`mission_logic` 包含两个主要节点：

```text
mission_node
magnetic_field_node
```

`mission_node` 是任务状态机。它读取机器狗定位和接收器读数，决定下一步探测动作，并发布目标点给机器狗。

任务流程大致是：

```text
搜索磁信号
  -> 根据左右箭头横向居中
  -> 在线上记录测点
  -> 根据管线方向向前跟踪
  -> 丢失信号时旋转/移动重捕获
  -> 到达边界或终点后结束
```

`magnetic_field_node` 是模拟接收器节点。它根据虚拟管线配置和机器狗定位生成模拟 `/magnetic_field`，用于在没有真实接收器时做 demo。

`mission_logic_msgs` 定义任务所需的接收器消息：

```text
mission_logic_msgs/msg/SensorMsg
```

## 编译

在工作区根目录执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

新终端中重新加载环境：

```bash
source install/setup.bash
```

## 接口约定

`mission_node` 订阅：

```text
/state_estimation   nav_msgs/msg/Odometry
/magnetic_field     mission_logic_msgs/msg/SensorMsg
```

`mission_node` 发布：

```text
/goal_pose          geometry_msgs/msg/PoseStamped
/speed              std_msgs/msg/Float32
```

`magnetic_field_node` 订阅：

```text
/state_estimation   nav_msgs/msg/Odometry
```

`magnetic_field_node` 发布：

```text
/magnetic_field     mission_logic_msgs/msg/SensorMsg
/pipeline_marker    visualization_msgs/msg/Marker
/receiver_marker    visualization_msgs/msg/Marker
```

机器狗侧需要满足：

```text
提供 /state_estimation
执行 /goal_pose
```

如果机器狗或接收器实际 topic 名不同，可以通过 launch 参数 remap。

## 接收器消息

消息定义在：

```text
mission_logic_msgs/msg/SensorMsg.msg
```

字段包括：

```text
magnetic_field
magnetic_field_covariance
signal_strength
depth_meters
current_milliamps
pipeline_heading_degrees
signal_strength_percent
left_arrow
right_arrow
```

当前任务逻辑尤其依赖：

```text
left_arrow
right_arrow
pipeline_heading_degrees
signal_strength
depth_meters
current_milliamps
```

其中 `left_arrow` 和 `right_arrow` 用于判断接收器相对管线的位置：

```text
只有 left_arrow      -> 需要向一侧横移
只有 right_arrow     -> 需要向另一侧横移
left_arrow/right_arrow 都为 true -> 认为接收器已经在线上
两个都为 false       -> 认为丢失管线，需要重新搜索
```

## Demo 1：真实定位 + 模拟管线和磁场

这一版用于机器狗已经能提供 `/state_estimation`、执行 `/goal_pose`，但还没有接入真实接收器的情况。

启动：

```bash
ros2 launch mission_logic demo_sim_signal.launch.py
```

数据流：

```text
机器狗 /state_estimation
  -> magnetic_field_node 生成模拟 /magnetic_field
  -> mission_node 根据读数规划
  -> /goal_pose
  -> 机器狗执行目标点
```

默认虚拟管线配置：

```text
mission_logic/config/straight_wire.json
```

默认任务参数：

```text
mission_logic/config/mission_logic.yaml
```

指定其他管线文件：

```bash
ros2 launch mission_logic demo_sim_signal.launch.py \
  pipeline_config_file:=mission_logic/config/straight_wire.json
```

如果 topic 名需要 remap：

```bash
ros2 launch mission_logic demo_sim_signal.launch.py \
  state_estimation_topic:=/state_estimation \
  magnetic_field_topic:=/magnetic_field \
  goal_pose_topic:=/goal_pose \
  speed_topic:=/speed
```

## Demo 2：真实定位 + 真实接收器信号

这一版用于真实接收器已经接入，并且能够发布 `mission_logic_msgs/msg/SensorMsg` 的情况。

启动：

```bash
ros2 launch mission_logic demo_real_signal.launch.py
```

这一版不会启动 `magnetic_field_node`，因此不会生成模拟磁场。真实接收器或桥接节点必须发布：

```text
/magnetic_field     mission_logic_msgs/msg/SensorMsg
```

数据流：

```text
机器狗 /state_estimation
真实接收器 /magnetic_field
  -> mission_node 根据读数规划
  -> /goal_pose
  -> 机器狗执行目标点
```

如果 topic 名需要 remap：

```bash
ros2 launch mission_logic demo_real_signal.launch.py \
  state_estimation_topic:=/state_estimation \
  magnetic_field_topic:=/magnetic_field \
  goal_pose_topic:=/goal_pose \
  speed_topic:=/speed
```

## 通用 Launch

也可以使用通用入口：

```bash
ros2 launch mission_logic mission_logic.launch.py
```

可用参数：

```text
use_sim_magnetic_field
start_mission_node
pipeline_config_file
```

示例：

```bash
ros2 launch mission_logic mission_logic.launch.py use_sim_magnetic_field:=true
ros2 launch mission_logic mission_logic.launch.py use_sim_magnetic_field:=false
ros2 launch mission_logic mission_logic.launch.py start_mission_node:=false
```

## 关键参数

主要参数文件：

```text
mission_logic/config/mission_logic.yaml
```

真机 demo 前建议重点检查：

```text
goal_frame
speed
workspace_min_x
workspace_max_x
workspace_min_y
workspace_max_y
search_probe_distance
centering_step
forward_step
reacquire_probe_offset
arrival_tolerance_distance
arrival_tolerance_yaw_degree
receiver_robot_dx
receiver_robot_dy
receiver_robot_same_direction
```

`receiver_robot_dx`、`receiver_robot_dy` 表示接收器相对机器狗本体的位置偏移。Demo 1 中，`mission_node` 和 `magnetic_field_node` 里的这几个接收器参数必须保持一致。

当前代码不做 TF 坐标变换，默认 `/state_estimation`、`/goal_pose` 和虚拟管线坐标处在同一个坐标系中。`goal_frame` 默认是：

```text
map
```

## 常用检查命令

```bash
ros2 topic hz /state_estimation
ros2 topic hz /magnetic_field
ros2 topic echo /goal_pose
ros2 topic echo /speed
ros2 interface show mission_logic_msgs/msg/SensorMsg
```

查看节点：

```bash
ros2 node list
ros2 node info /mission_node
ros2 node info /magnetic_field_node
```

## 真机运行建议

第一次上机器狗时，建议先用 Demo 1 验证完整闭环：

```text
真实定位 -> 模拟接收器 -> 任务规划 -> /goal_pose -> 机器狗执行
```

确认 `/goal_pose` 行为安全后，再切换到 Demo 2：

```text
真实定位 -> 真实接收器 -> 任务规划 -> /goal_pose -> 机器狗执行
```

初次测试建议在 `mission_logic/config/mission_logic.yaml` 中调小：

```text
speed
forward_step
centering_step
search_probe_distance
reacquire_probe_offset
```

并限制：

```text
workspace_min_x
workspace_max_x
workspace_min_y
workspace_max_y
```

这样即使接收器读数异常，机器狗也只会在较小范围内尝试搜索和重捕获。
