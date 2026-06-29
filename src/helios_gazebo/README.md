# helios_gazebo

Helios 底盘 **Gazebo 仿真**包，用于在没有真机时，于 Linux + RViz 中跑通建图与导航。

配合 `helios_description`（机器人模型）和 `helios_nav`（雷达预处理/配置）使用。
仿真与真机共用同一套话题契约（`/scan`、`/odom`、`/cmd_vel`、TF），上层算法不变。

---

## 1. 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo Classic 11（`gazebo_ros_pkgs`）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro ros-humble-rviz2 \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

## 2. 编译

把 `helios_description`、`helios_gazebo`、`helios_nav` 三个包放到同一工作空间 `src/` 下：

```bash
cd ~/helios_nav_ws
colcon build --symlink-install
source install/setup.bash      # zsh 用 setup.zsh
```

## 3. 三种运行方式（由简到全）

### 3.1 只看模型（最快验证 URDF/TF，无需 Gazebo）

```bash
ros2 launch helios_description display.launch.py
```
RViz 里应看到底盘方块 + 前后两个红色雷达，TF 树 `base_footprint → base_link → 两个 laser_link`。

### 3.2 仿真基础环境（Gazebo + 雷达 + 遥控）

```bash
ros2 launch helios_gazebo sim.launch.py
# 另开终端遥控移动：
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
验证：
```bash
ros2 topic hz /scan        # 合并后的雷达
ros2 topic echo /odom --once
ros2 topic list | grep sr_amr_control   # 仿真发布的前后雷达原始话题
```

### 3.3 仿真 + SLAM 建图

```bash
ros2 launch helios_gazebo sim_slam.launch.py
# 遥控走一圈，RViz 里看地图增长
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 保存地图：
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_sim
```

### 3.4 仿真 + SLAM + Nav2 自主导航

```bash
ros2 launch helios_gazebo sim_nav.launch.py
# RViz 里用 "Nav2 Goal" 点目标点，机器人自主规划行驶
```

---

## 4. 话题契约（仿真 vs 真机）

| 话题 / TF | 仿真（本包） | 真机（helios_nav + SDK） |
|---|---|---|
| `/sr_amr_control/front\|rear/scan` | Gazebo 雷达插件 | 厂家 SDK |
| `/scan` | laser_preprocess 合并 | laser_preprocess 合并 |
| `/odom` + `odom→base_footprint` | Gazebo planar_move | odom_publisher 速度积分 |
| `/cmd_vel` 订阅 | Gazebo planar_move | cmd_vel_relay → SDK |
| `base_link→laser` TF | URDF | URDF |

因为两边话题一致，`helios_nav` 的 slam/amcl/nav2 配置可直接复用。

---

## 5. 待厂家确认后要替换的占位参数

| 文件 | 参数 | 说明 |
|---|---|---|
| `helios_description/urdf/helios.urdf.xacro` | `lidar_front_*` / `lidar_rear_*` | 雷达真实安装位姿 |
| 同上 | `base_length/width/height` | 底盘真实尺寸 |
| `helios_nav/config/nav2_params.yaml` | `footprint`、`vx/vy/wz_max` | 真实外形与速度上限 |
```
