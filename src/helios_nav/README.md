# helios_nav

Helios 轮臂机器人底盘 2D 建图与导航包。

技术栈：**slam_toolbox + map_server + amcl + nav2**（全向底盘局部控制用 MPPI Omni）。
作用：对接底盘 SDK（`sr_amr_control`）的雷达与速度接口，运行开源导航算法栈，替代厂家 MATRIX。

> 说明：**odom** 已由 `odom_publisher` 从 `system_state` 速度积分生成；**真实雷达 TF 外参 / IMU** 仍待实测确认。

---

## 1. 目录结构

```
helios_nav/
├── helios_nav/
│   ├── laser_preprocess.py   # 雷达 101.0->inf 过滤；可选合并前后雷达 -> /scan
│   ├── odom_publisher.py     # system_state 速度积分 -> /odom + TF
│   └── cmd_vel_relay.py      # /cmd_vel -> 底盘 remote_control_cmd_vel（自动开启远程控制）
├── config/
│   ├── slam_toolbox.yaml     # 建图参数
│   ├── nav2_params.yaml      # amcl + MPPI(Omni) + costmap + planner 等
│   └── ekf.yaml              # 可选：拿到 IMU 后融合轮速+IMU
├── launch/
│   ├── static_tf.launch.py   # 占位雷达 TF（待填真实外参）
│   ├── bringup.launch.py     # 预处理 + cmd_vel 中继 + TF
│   ├── mapping.launch.py     # bringup + slam_toolbox
│   ├── localization.launch.py# bringup + map_server + amcl
│   └── navigation.launch.py  # localization + Nav2 全栈
└── maps/                     # 保存的地图
```

---

## 2. 部署（在 Orin 上）

```bash
# 依赖（一次性）
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-robot-localization

# 放到工作空间并编译
mkdir -p ~/helios_nav_ws/src
# 把本包拷到 ~/helios_nav_ws/src/helios_nav
cd ~/helios_nav_ws
colcon build --packages-select helios_nav
source install/setup.bash   # zsh 用 setup.zsh
```

底盘 SDK 节点需先启动（含雷达）：

```bash
ros2 launch sr_amr_control amr_control.launch.py \
  connect_ip:=192.168.71.50 lidar:=true
```

---

## 3. 开发/测试前必须确认的接口

| 项 | 命令 | 影响 |
|---|---|---|
| odom 话题 | `ros2 topic echo /odom --once` | 由 odom_publisher 发布，需 system_state 有数据 |
| TF 树 | `ros2 run tf2_tools view_frames` | 需要 map→odom→base_link→laser |
| 雷达 frame | `ros2 topic echo /sr_amr_control/front/scan --once` | 决定 static_tf 的 child frame |
| IMU（可选） | `ros2 topic list -t | grep -i imu` | 启用 ekf.yaml 才需要 |

- `odom_publisher` 为速度积分里程计，会有漂移；amcl/slam 会用激光修正。
- 确认 **真实雷达外参** 后，修改 `launch/static_tf.launch.py` 里的 x/y/z/yaw。

---

## 4. 使用流程

### 4.1 建图

```bash
ros2 launch helios_nav mapping.launch.py
# 遥控机器人走完场地后保存地图：
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_map
```

### 4.2 定位（验证 amcl）

```bash
ros2 launch helios_nav localization.launch.py map:=/home/admin/maps/helios_map.yaml
# RViz 用 2D Pose Estimate 给初始位姿
```

### 4.3 导航

```bash
ros2 launch helios_nav navigation.launch.py map:=/home/admin/maps/helios_map.yaml
# RViz 先 2D Pose Estimate，再 Nav2 Goal 发目标点
```

---

## 5. 关键参数位置

| 需求 | 文件 | 参数 |
|---|---|---|
| 合并双雷达 | launch | `merge:=true` |
| 雷达无效值 | nav2/preprocess | `invalid_value: 101.0` |
| 全向运动模型 | nav2_params.yaml | amcl `OmniMotionModel`、MPPI `motion_model: Omni` |
| 限速（初期保守） | nav2_params.yaml | `vx_max/vy_max/wz_max` |
| 机器人外形 | nav2_params.yaml | `footprint` |
| 坐标系名 | 各 yaml | `base_link`（如底盘用 base_footprint 需统一改） |

---

## 6. 注意事项（轮臂机器人）

- 导航时双臂应回收至安全姿态，footprint 已留余量但不覆盖手臂伸出。
- 初期限速 0.2~0.4 m/s，验证稳定后再提。
- 2D 激光有盲区，后续建议接入 TOF 补低矮/盲区障碍。
- `merge=true` 依赖正确 TF；外参未确认前可先用单雷达（`merge=false`）跑通流程。
