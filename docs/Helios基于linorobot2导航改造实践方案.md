# Helios 轮臂机器人 · 基于 linorobot2 的 2D 导航改造实践方案

> 目的：以 linorobot2（humble 分支）为规范骨架，移植 Helios 专用的「桥接 + 双雷达 + 全向」改造，做出一套可维护、可扩展的底盘导航工程。
> 路线：先在 Gazebo 仿真跑通 → 再上 Orin 真机；缺的精度类数据后续找厂家补。
> 本文档为「实践参考」，按章节顺序执行即可。
> 创建：2026-06-29

---

## 0. 一句话总览

```
linorobot2 骨架（EKF + Nav2/slam launch + rviz + maps + Gazebo Classic）
        +
Helios 器官移植（3 个 SDK 桥接节点 + 双雷达 URDF + 全向 nav2 参数）
        =
helios_linorobot_ws（新工作空间，现有 helios_nav_ws 保留作对照）
```

为什么这么做：linorobot2 本身的底层是 micro-ROS + 自制硬件，**和珞石 SDK 完全不通**；它能白送给我们的是「工程结构 + EKF + launch 范式」，而 Helios 真正难的「SDK 桥接」必须自己写（已在 helios_nav 写好，搬过去即可）。

---

## 1. 导航所需数据清单（有 / 没有 / 存在形式）

> 这是整个方案的地基。先认清「手里有什么、缺什么、缺的怎么绕」，再动手。

### 1.1 全量数据表

| # | 数据 | Nav2 用途 | 必需性 | Helios 有无 | 当前存在形式 / 来源 |
|---|------|-----------|--------|-------------|---------------------|
| 1 | 激光 `/scan` (`sensor_msgs/LaserScan`) | 建图、AMCL 匹配、障碍层 | 必需 | ✅ 有 | 双雷达 `/sr_amr_control/front\|rear/scan`，360°/1°/0.1~25m/**5Hz**，无效值 `101.0`，经 `laser_preprocess` 过滤合并→`/scan` |
| 2 | 里程计 `/odom` (`nav_msgs/Odometry`) | 连续位姿、TF `odom→base` | 必需 | ⚠️ 无原生 | SDK 只给 `system_state` 速度 vx/vy/wz（**3.33Hz**），`odom_publisher` 积分生成（会漂移、无 IMU 纠偏） |
| 3 | TF `odom→base_footprint` | 所有定位/规划基准 | 必需 | ⚠️ 自生成 | `odom_publisher` 随 `/odom` 一起广播（也可交给 EKF 发布，见 §6） |
| 4 | TF `base_link→各雷达`（外参） | 多雷达点云对齐 | 必需 | ❌ 缺真值 | URDF 用**拟合占位值** `(±0.30, ∓0.28, 0.16)`，待厂家替换 |
| 5 | TF `map→odom` | 全局定位纠偏 | 必需 | ✅ 算法产生 | slam_toolbox（建图）/ AMCL（定位）输出，非传感器数据 |
| 6 | 速度指令 `/cmd_vel` (`Twist`) | Nav2 控制输出 | 必需 | ✅ 有通道 | `cmd_vel_relay` → `/sr_amr_control/remote_control_cmd_vel` + 使能服务 |
| 7 | 机器人 footprint / 尺寸 | costmap 碰撞 | 必需 | ⚠️ 估计值 | 手册 0.68×0.64 + 估计高度 |
| 8 | 速度/加速度上限 vx/vy/wz max | 限速、轨迹采样 | 必需 | ⚠️ 保守估计 | 暂填 vx 0.4 / vy 0.4 / wz 1.0 |
| 9 | IMU (`sensor_msgs/Imu`) | EKF 融合、稳定 yaw | 可选(强推) | ❌ 无 | `ekf.yaml` 已预留，拿到即启用 |
| 10 | 轮速编码器 | 高精度 odom | 可选 | ❌ 无 | SDK 不暴露，用速度积分替代 |
| 11 | TOF 低障数据（点云/深度） | 补激光盲区、矮障碍 | 可选 | ⚠️ 硬件有/接口未知 | 4 个 TOF（前后左右，地面 5cm~1m），无 ROS2 demo；可 `pointcloud_to_laserscan` 接入 |
| 12 | 地图 `/map` (`OccupancyGrid`) | 全局规划、AMCL | 导航阶段必需 | ✅ 可产出 | slam_toolbox 建图 → `map_saver` 存 pgm/yaml |
| 13 | 初始位姿 `/initialpose` | AMCL 初始化 | 定位阶段必需 | ✅ 手动 | RViz「2D Pose Estimate」；或参照 `system_state.current_pose` |
| 14 | `/clock` | 仿真时间同步 | 仿真必需 | ✅ | Gazebo 提供，`use_sim_time:=true` |

### 1.2 按状态归类

**✅ 已具备（可直接跑）**：`/scan`、`/cmd_vel`、速度反馈、`map→odom`、`/map`、急停。

**⚠️ 有但打折/占位（能跑需改善）**：`/odom`（积分漂移）、雷达外参（占位）、footprint（估计）、速度上限（猜测）、TOF（接口未知）。

**❌ 完全缺失（需厂家或绕过）**：IMU、轮速编码器、TOF ROS2 接口。

### 1.3 结论

- **导航能跑的最小集合已齐**（scan + cmd_vel + 自生成 odom + 算法 TF），仿真和真机都能起。
- **缺的是「精度类」数据**：原生 odom、IMU、真实外参/footprint/限速 —— 不补则精度和稳定性打折。
- **TOF 是增强项**，不影响主流程。

---

## 2. 向厂家确认清单（按优先级）

| 优先级 | 问题 | 为什么关键 |
|--------|------|-----------|
| P0 | 能否直接提供标准 `nav_msgs/Odometry`？频率多少？ | 决定 odom 质量，影响整个定位 |
| P0 | `base_link` → 前/后雷达**真实外参**（xyz + yaw） | 双雷达匹配/建图正确性 |
| P0 | 底盘真实 footprint + 各传感器安装高度 | costmap 碰撞 |
| P1 | 全向 vx/vy/wz **最大速度 + 加速度** | Nav2 限速、MPPI 采样 |
| P1 | 能否提供 **IMU** 话题？类型/频率？ | 定位稳定性、EKF |
| P2 | 4 个 TOF 是否有 **ROS2 接口**（点云/深度）+ 外参 | 矮障碍避障 |
| P2 | `system_state.current_pose` 坐标系定义与精度 | 可作真值/初始位姿参照 |

> 拿到 P0 即可显著提升；P1 提精度；P2 做增强。这些参数回填时只改 URDF / static_tf / nav2 参数，不动代码逻辑。

---

## 3. 算法栈（不变，linorobot2 与 helios 一致）

| 阶段 | 组件 | Helios 配置 |
|------|------|-------------|
| 建图 | SLAM Toolbox（async） | 在线 2D 激光 SLAM |
| 定位 | map_server + AMCL | **运动模型 OmniMotionModel** |
| 全局规划 | Nav2 Smac Planner 2D | 静态地图规划 |
| 局部控制 | Nav2 MPPI Controller | **motion_model: Omni**，支持 vx/vy/wz |
| 里程计融合 | robot_localization EKF | 轮速 + IMU（IMU 到位后启用） |
| 代价地图 | Obstacle + Inflation Layer | 用 `/scan` 检测障碍 |

> 这些算法包都在 apt 里（`ros-humble-navigation2`、`ros-humble-slam-toolbox`、`ros-humble-robot-localization`），不进我们的 repo。

---

## 4. 关键认知：linorobot2 与 Helios 的插件天然一致

| 项 | linorobot2 humble | Helios 现状 | 结论 |
|----|-------------------|-------------|------|
| 全向底盘插件 | `libgazebo_ros_planar_move.so` | 同一个 | ✅ 全向现成 |
| 雷达插件 | `libgazebo_ros_ray_sensor.so`（参数化宏） | 同一个 | ✅ 双雷达=宏实例化两次 |
| 全向整车模板 | `mecanum.urdf.xacro`（4 轮 + omni_drive） | — | ✅ 直接有全向模板 |
| 仿真器 | Gazebo Classic | Gazebo Classic | ✅ 一致 |
| EKF | `robot_localization`（已配好） | 占位 | ✅ 白捡 |

**=> 「换全向轮」在 linorobot2 上几乎零改（它本来就有全向）；「单雷达→双雷达」是宏实例化两次 + 合并节点。**

### linorobot2 帮不了、必须自己写的部分（Helios 专用）

| Helios 必需 | linorobot2 有吗 | 谁来做 |
|-------------|-----------------|--------|
| `system_state` 速度积分→`/odom` | ❌ | `odom_publisher.py`（已有） |
| `/cmd_vel`→`remote_control_cmd_vel`+使能 | ❌ | `cmd_vel_relay.py`（已有） |
| 双雷达 `101.0` 过滤合并 | ❌ | `laser_preprocess.py`（已有） |
| 急停服务对接 | ❌ | 后续补 |

---

## 5. 目标工程结构

```
helios_linorobot_ws/
└── src/
    ├── linorobot2_navigation/   # 保留：slam/nav2 launch、rviz、maps、config
    ├── linorobot2_gazebo/       # 保留：world + gazebo.launch 模式
    ├── linorobot2_base/         # 只留 config/ekf.yaml，其余删
    ├── helios_description/      # 用我们的：双雷达 + 全向 URDF（替换 linorobot2_description）
    └── helios_bringup/          # 我们的 3 个桥接节点 + bringup.launch（替换 linorobot2_bringup）
```

| linorobot2 包 | 处理 |
|---------------|------|
| `linorobot2_navigation` | 保留（改 nav2 参数为 Omni） |
| `linorobot2_gazebo` | 保留（spawn 换成 helios urdf） |
| `linorobot2_base` | 只留 `ekf.yaml` |
| `linorobot2_bringup` | 删 → `helios_bringup` 取代 |
| `linorobot2_description` | 删 → `helios_description` 取代 |

---

## 6. 坐标系（TF）约定

```
map → odom → base_footprint → base_link → {front_laser_link, rear_laser_link, imu_link, tof_*}
```

| TF | 发布者 | 备注 |
|----|--------|------|
| `map → odom` | slam_toolbox（建图）/ AMCL（定位） | 互斥，不同时跑 |
| `odom → base_footprint` | **二选一**：EKF（有 IMU 时）或 `odom_publisher` | 同一个 TF 只能一个发布者！ |
| `base_footprint → base_link` 及传感器 | robot_state_publisher（URDF） | 静态 |

> ⚠️ 统一用 `base_footprint`（跟 linorobot2）。把 `odom_publisher` / nav2 参数里的 `base_frame` 都改成 `base_footprint`。
> ⚠️ EKF 与 odom_publisher 不能同时发 `odom→base_footprint`：未接 IMU 时用 odom_publisher 发 TF；接入 EKF 后让 odom_publisher 只发 `/odom`（`publish_tf=false`），由 EKF 发 TF。

---

## 7. 分步实施

### 步骤 0：建并行工作空间（不动现有 helios_nav_ws）

```bash
mkdir -p ~/helios_linorobot_ws/src && cd ~/helios_linorobot_ws/src
git clone -b humble https://github.com/linorobot/linorobot2.git
cp -r ~/helios_nav_ws/src/helios_description .
cp -r ~/helios_nav_ws/src/helios_nav ./helios_bringup    # 改名为桥接包
```

### 步骤 1：装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard
```

### 步骤 2：包取舍（按 §5 删/留）

删除 `linorobot2_bringup`、`linorobot2_description`；`linorobot2_base` 只保留 `config/ekf.yaml`。

### 步骤 3：双雷达 URDF（linorobot laser 宏实例化两次）

```xml
<!-- 前雷达 -->
<xacro:laser frame_id="front_laser_link" topic_name="/sr_amr_control/front/scan"
             ray_count="360" min_angle="-3.14159" max_angle="3.14159"
             min_range="0.05" max_range="25.0" update_rate="5">
  <origin xyz="0.30 -0.28 0.16" rpy="0 0 0"/>   <!-- 占位，待厂家外参 -->
</xacro:laser>

<!-- 后雷达 -->
<xacro:laser frame_id="rear_laser_link" topic_name="/sr_amr_control/rear/scan"
             ray_count="360" min_angle="-3.14159" max_angle="3.14159"
             min_range="0.05" max_range="25.0" update_rate="5">
  <origin xyz="-0.30 0.28 0.16" rpy="0 0 0"/>   <!-- 占位，待厂家外参 -->
</xacro:laser>
```

仿真里两雷达发 front/rear scan → `laser_preprocess` 合并过滤 → `/scan`。

### 步骤 4：全向（用 linorobot 现成 omni_drive_controller）

- 整车参考 `mecanum.urdf.xacro`：4 轮 + `<xacro:omni_drive_controller/>`
- 插件即 `libgazebo_ros_planar_move.so`，与现有 helios_gazebo 一致

### 步骤 5：改 nav2 参数为全向（关键，漏一处就跑偏）

`linorobot2_navigation/config/navigation.yaml`（及 `navigation_sim.yaml`）：

```yaml
amcl:
  ros__parameters:
    base_frame_id: "base_footprint"
    robot_model_type: "nav2_amcl::OmniMotionModel"      # 原 Differential

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"     # 原 DWB/RPP
      motion_model: "Omni"
      vx_max: 0.4
      vx_min: -0.3
      vy_max: 0.4        # 全向横移（差速为 0）
      wz_max: 1.0
```

> 这些段可直接从 `helios_nav/config/nav2_params.yaml` 复制（已调好 Omni + MPPI + critics）。
> costmap 的 `robot_radius`/`footprint` 改成 Helios 实际尺寸。

### 步骤 6：bringup 接桥接节点

把 `helios_bringup/launch/bringup.launch.py` 写成（去掉一切 micro-ROS / lasers 驱动）：

```python
Node(package="helios_bringup", executable="laser_preprocess",
     parameters=[{"front_topic": "/sr_amr_control/front/scan",
                  "rear_topic": "/sr_amr_control/rear/scan",
                  "output_topic": "/scan", "merge": True,
                  "target_frame": "base_footprint",
                  "invalid_value": 101.0, "range_min": 0.05, "range_max": 25.0}]),

Node(package="helios_bringup", executable="odom_publisher",
     parameters=[{"system_state_topic": "/sr_amr_control/system_state",
                  "odom_topic": "/odom", "odom_frame": "odom",
                  "base_frame": "base_footprint",
                  "publish_tf": True}]),        # 接 EKF 后改 False

Node(package="helios_bringup", executable="cmd_vel_relay",
     parameters=[{"in_topic": "/cmd_vel",
                  "out_topic": "/sr_amr_control/remote_control_cmd_vel",
                  "enable_on_start": True, "oba_on_start": True}]),

# 可选：EKF（拿到 IMU 后启用，用 linorobot2_base/config/ekf.yaml）
# Node(package="robot_localization", executable="ekf_node",
#      parameters=[ekf_yaml])
```

### 步骤 7：gazebo spawn 换成 helios 模型

`linorobot2_gazebo/launch/gazebo.launch.py` 里 spawn 的 URDF 路径指向 `helios_description` 的 xacro。

### 步骤 8：编译 + 仿真验证

```bash
cd ~/helios_linorobot_ws && colcon build --symlink-install
source install/setup.bash

# 仿真起 Gazebo
ros2 launch linorobot2_gazebo gazebo.launch.py
# 建图（slam + nav2 同时，sim 时间）
ros2 launch linorobot2_navigation slam.launch.py sim:=true rviz:=true
# 遥控走一圈
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 存图
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_sim
# 已知地图导航
ros2 launch linorobot2_navigation navigation.launch.py map:=~/maps/helios_sim.yaml sim:=true rviz:=true
```

### 步骤 9：真机验证（Orin 192.168.71.51 / 底盘 192.168.71.50）

```bash
# 1) 起厂家 SDK（含雷达）
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=192.168.71.50 lidar:=true
# 2) 起桥接层
ros2 launch helios_bringup bringup.launch.py
# 3) 建图 / 导航（use_sim_time:=false）
ros2 launch linorobot2_navigation slam.launch.py rviz:=true
ros2 launch linorobot2_navigation navigation.launch.py map:=/home/admin/maps/helios_map.yaml
```

---

## 8. 验收检查点

| 检查 | 期望 | 排查 |
|------|------|------|
| `ros2 topic echo /scan` | 有数据、无 101.0、点数合理 | 看 laser_preprocess 日志 |
| `ros2 run tf2_tools view_frames` | `map→odom→base_footprint→雷达` 完整 | 确认无双重发布 TF |
| RViz 看 `/scan` | 与机器人朝向一致、双雷达拼接对 | 改雷达外参 origin |
| 建图 | 边走边出图，闭环正常 | slam_toolbox 参数 / odom 漂移 |
| `2D Pose Estimate` 后 AMCL | 粒子收敛、激光贴合地图 | OmniMotionModel 是否生效 |
| `Nav2 Goal` | 规划紫线、机器人跟随 | MPPI motion_model=Omni、限速 |
| 全向横移 | 能 vy 横走（非只前进+转） | 确认 amcl/controller 都是 Omni |

---

## 9. 必改 4 处速记（最容易漏）

1. **AMCL** `robot_model_type: OmniMotionModel`
2. **Controller** MPPI `motion_model: Omni` + `vy_max>0`
3. **base_frame** 统一 `base_footprint`（桥接节点 + nav2 参数）
4. **TF 唯一发布者**：odom_publisher 与 EKF 不同时发 `odom→base_footprint`

---

## 10. 演进路线

- [ ] 仿真跑通：建图 → 存图 → 定位 → 导航（全向横移验证）
- [ ] 厂家补 P0：原生 odom / 雷达外参 / footprint → 替换占位值
- [ ] 真机跑通同一套上层（只换底层 SDK 桥接）
- [ ] 拿到 IMU → 启用 robot_localization EKF（odom_publisher 改 publish_tf=false）
- [ ] TOF 可用 → `pointcloud_to_laserscan` 接入 costmap 补激光盲区
- [ ] 对接公司平台：任务下发、状态回传、视频流

---

## 11. 环境信息速查

| 项 | 值 |
|----|----|
| Orin 算力机 | `admin@192.168.71.51` |
| 底盘 / MATRIX | `192.168.71.50` |
| ROS 版本 | Humble（Orin 默认 zsh，用 setup.zsh） |
| 完整 SDK | `/home/admin/agvsdk/standard_robots_amr_ros2-v1.3.0` |
| 仿真器 | Gazebo Classic 11 |
| 新工作空间 | `~/helios_linorobot_ws` |
| 对照工作空间 | `~/helios_nav_ws`（保留不动） |
| 参考代码 | `d:\chx\lios\_ref\linorobot2_humble`、`_ref\agilex_open_class` |

---

## 12. 参考来源

- linorobot2（humble）：https://github.com/linorobot/linorobot2/tree/humble
- 松灵 ROS2 开讲啦配套：https://github.com/agilexrobotics/agilex_open_class
- Nav2 文档：https://docs.nav2.org/
- SLAM Toolbox：https://github.com/SteveMacenski/slam_toolbox
- robot_localization：https://github.com/cra-ros-pkg/robot_localization
