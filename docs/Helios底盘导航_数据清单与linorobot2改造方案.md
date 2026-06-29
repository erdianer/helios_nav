# Helios 轮臂机器人底盘导航 —— 数据清单 + 基于 linorobot2 的改造方案

> 用途：作为**实践参考**。上半部分说明"底盘导航需要哪些数据、现在有没有、以什么形式呈现、怎么用（桥接）"；下半部分给出**基于 linorobot2 改造**成 Helios 底盘导航包的具体方案与步骤。
> 技术栈：slam_toolbox + map_server + amcl + nav2（全向底盘，局部控制 MPPI Omni）。
> 环境：ROS2 Humble、Orin 算力机 `192.168.71.51`、底盘 `192.168.71.50`、完整 SDK `sr_amr_control` v1.3.0。
> 关联文档：《雷达测试.md》（真机实测）、《底盘2D导航开源方案调研.md》、《Helios底盘2D导航方案总结.md》。
> 最后更新：2026-06-29

---

## 一、名词约定：什么是"桥接"

厂家底盘说的是"私有方言"（`sr_amr_control` SDK 的话题/服务），开源导航栈（Nav2/slam_toolbox/amcl）认的是"ROS 标准接口"。**桥接 = 中间一层薄适配节点，不改 SDK、也不改 Nav2，只做话题/坐标系/命令的翻译。**

```
厂家 SDK (sr_amr_control)        桥接层 (自写 Python 节点)          开源导航栈
────────────────────────        ─────────────────────────         ──────────────
front/scan, rear/scan      ──▶  laser_preprocess          ──▶  /scan   ──▶ slam_toolbox / amcl / costmap
system_state(vx,vy,wz)     ──▶  odom_publisher (+EKF)     ──▶  /odom + TF ─▶ amcl / Nav2
/cmd_vel (Nav2 输出)        ──▶  cmd_vel_relay            ──▶  remote_control_cmd_vel ─▶ 底盘运动
```

---

## 二、底盘导航数据清单（核心参考表）

> 列含义：**导航是否必需** / **现在有没有** / **以什么形式呈现** / **怎么用** / **桥接或处理方式** / **状态**。

### 2.1 必需数据（缺了导航跑不起来）

| 数据项 | 必需 | 现状 | 呈现形式（话题 / 类型 / 来源） | 用途 | 桥接 / 处理方式 | 状态 |
|---|---|---|---|---|---|---|
| **激光 /scan** | ✅ | 有（双雷达） | `/sr_amr_control/front/scan`、`/sr_amr_control/rear/scan`（`sensor_msgs/LaserScan`，360°/1°/721 bin/约 5Hz，量程 0–100m） | 建图、定位、代价地图避障 | `laser_preprocess`：把无效占位 `101.0`→`inf`；可选合并双雷达为单帧 `/scan` | 真机已验证 |
| **里程计 /odom + TF odom→base_link** | ✅ | 部分（无现成 odom） | 仅 `system_state` 内有车体速度 `linear_velocity_x/y`、`angular_velocity`（约 3.33Hz） | amcl/slam 的运动先验；Nav2 局部规划 | `odom_publisher`：速度积分→标准 `/odom`+TF。**推荐再过一层 robot_localization EKF** 输出 `/odom` 与 `odom→base_footprint` | 桥接可发布，精度待验证 |
| **速度控制 /cmd_vel** | ✅ | 有（私有话题） | 底盘收 `/sr_amr_control/remote_control_cmd_vel`（`geometry_msgs/Twist`），需先开远程控制服务 | 执行 Nav2 输出的运动命令 | `cmd_vel_relay`：`/cmd_vel`→`remote_control_cmd_vel`；启动时调 `remote_control_enabled`（`std_srvs/SetBool`） | 桥接已写 |
| **雷达外参 TF base_link→雷达** | ✅ | 无真值（占位） | 应为静态 TF：`base_link→right_front_laser_link` / `left_behind_laser_link` | 决定 scan 在车体的位置，直接影响建图/合并质量 | 现用 `static_tf.launch.py` 占位值（前 +0.30 / 后 -0.30）；测试期可让 SDK 把 frame 设成 `map` 跳过 TF | **必须找厂家要真实安装位姿** |
| **TF 完整树** map→odom→base_footprint→base_link→雷达 | ✅ | 部分 | 由 amcl(map→odom)+EKF/odom(odom→base)+URDF(base→雷达) 共同构成 | 所有定位/导航/可视化的基础 | URDF（`robot_state_publisher`）+ EKF + amcl 各发一段 | 链路设计已定，需打通 |
| **栅格地图 map** | ✅ | 建图后产出 | `nav_msgs/OccupancyGrid`（`.pgm`+`.yaml`） | 全局定位与全局规划的底图 | slam_toolbox 建图→`map_saver_cli` 保存→`map_server` 加载 | 流程已就绪 |
| **机器人外形 footprint / 尺寸** | ✅ | 有手册值 | 底盘 0.68×0.64m（手册）；URDF/nav2 参数里配置 | 碰撞检查、膨胀层、贴边通过性 | 写入 URDF 和 `nav2_params.yaml` 的 footprint | 用手册值，待厂家确认精确值与含臂包络 |

### 2.2 可选 / 增强数据（有则更稳，无也能先跑）

| 数据项 | 必需 | 现状 | 呈现形式 | 用途 | 桥接 / 处理方式 | 状态 |
|---|---|---|---|---|---|---|
| **IMU** | ⬜ 可选 | 未确认 | 期望 `sensor_msgs/Imu` | 融合进 EKF，提升旋转/打滑时定位稳定性 | 有则进 `ekf.yaml` 的 `imu0`；可选 `imu_filter_madgwick` 预处理 | 待向厂家确认 |
| **4 路 TOF 深度相机** | ⬜ 可选 | 硬件有，ROS 接口未知 | 期望点云/深度（前后左右，地面 5cm–1m） | 补 2D 激光盲区与低矮障碍 | `pointcloud_to_laserscan`（省事）或 STVL/nvblox 体素层接入 costmap | 待确认 SDK 接口 + 外参 |
| **厂家现成位姿（参照）** | ⬜ | 有 | `system_state.current_pose`（map 坐标） | 作真值参照、对比自研定位精度 | 仅读取比对，**不用于自研定位**（避免依赖 MATRIX） | 已验证 |
| **急停状态** | ⬜ 建议接 | 有 | `system_state.estop_active`、`emergency_stop`/`release` 服务 | 急停时清零速度、上报平台 | `odom_publisher` 已处理 estop 清零；可扩展上报 | 已部分处理 |
| **电池状态** | ⬜ | 手册提供 | `/sr_amr_control/battery_state` | 平台监控、低电返航 | 平台层订阅 | 平台对接阶段用 |

### 2.3 雷达数据已知限制（实践必读）

- **无效值 `101.0`**：表示该方向无回波，必须转 `inf`，否则被当成真实远距离障碍，污染地图与定位。
- **非原始 ring data**：SDK 从 SROS 的 `loc_laser_points`（处理后障碍点）重建 `LaserScan`，不是驱动层原始扫描。
- **只有 LaserScan，无 PointCloud2**。
- **默认关闭**：雷达上报默认 `lidar:=false`，重启节点后需重新开启（或 launch 时 `lidar:=true`）。
- **频率偏低**：约 5Hz、1° 分辨率；移动建图/避障效果需实测。

---

## 三、数据缺口与前置动作（开发前必做）

| 缺口 | 影响 | 现在的绕过 | 要做的事 |
|---|---|---|---|
| 雷达真实外参（base_link→前/后雷达） | 建图质量、双雷达合并 | URDF/static_tf 占位值 | 找厂家要 xyz/yaw，填入 URDF |
| 能否直接给 /odom | 里程计精度 | system_state 速度积分（3.33Hz、漂移） | 问厂家有无标准 odom 话题 |
| 能否给 IMU | 定位稳定性 | 无，ekf 预留 | 确认话题，启用 EKF 融合 |
| TOF ROS 接口 + 外参 | 低矮/盲区避障 | 暂不接 | 确认点云/深度话题与安装位姿 |
| 含机械臂的最大包络 | footprint 安全 | 用底盘 0.68×0.64 | 约定双臂回收姿态 + 实际包络 |

> 核查命令（Orin 上）：`ros2 topic list -t`、`ros2 run tf2_tools view_frames`、`ros2 topic echo /sr_amr_control/system_state --once`，并查 SDK 的 `README-zh.md`。

---

## 四、基于 linorobot2 的改造方案

### 4.1 为什么"改 linorobot2"而不是从零写

linorobot2 提供了一套**成熟的上层组织范式**值得复用；但它的**底层硬件层（micro-ROS 固件 + linorobot2_base）对 Helios 完全用不上**——Helios 是带 SDK 的成品 AMR，底层用我们自己的桥接节点替代。

| linorobot2 的部分 | 对 Helios | 处理 |
|---|---|---|
| `linorobot2_base` + micro-ROS 固件（发 `odom/unfiltered`、收 `cmd_vel`） | ❌ 用不上 | **删掉**，用桥接节点替代 |
| `robot_localization` EKF 链路（odom+IMU→/odom+TF） | ✅ 值得抄 | 保留，odom 输入改成桥接来的 `odom/unfiltered` |
| `navigation.launch.py` → include 官方 `nav2_bringup/bringup_launch.py` | ✅ 值得抄 | 保留，换成我们的 `nav2_params.yaml` |
| 分层 launch（bringup/sensors/description/slam/navigation） | ✅ 值得抄 | 保留骨架 |
| 默认差速参数 | ⚠️ 不合适 | 换成已有的 Omni 参数（amcl OmniMotionModel + MPPI Omni） |

**结论：混合方案 = linorobot2 的上层骨架 + Helios 桥接层（保留现有 3 个节点）+ 已调好的 Omni 参数。**

### 4.2 目标包结构（fork linorobot2 命名习惯）

```
helios_nav_ws/src/
├── helios_description/        # 机器人模型（仿真/真机共用）
│   ├── urdf/helios.urdf.xacro   # 全向底盘 + 双雷达 + 4 TOF 占位 + base_footprint
│   └── launch/description.launch.py   # robot_state_publisher
├── helios_base/              # 对接 SDK 的"底层"（替代 linorobot2_base + micro-ROS）
│   ├── helios_base/laser_preprocess.py   # 101.0→inf，可选合并
│   ├── helios_base/odom_publisher.py     # system_state→odom/unfiltered（不发 TF）
│   ├── helios_base/cmd_vel_relay.py      # /cmd_vel→remote_control_cmd_vel
│   └── config/ekf.yaml                   # robot_localization 融合配置
├── helios_bringup/           # 启动编排（仿 linorobot2_bringup）
│   └── launch/bringup.launch.py  # description + 桥接节点 + EKF + 静态外参 TF
└── helios_navigation/        # 建图 / 定位 / 导航（仿 linorobot2_navigation）
    ├── config/slam_toolbox.yaml
    ├── config/navigation.yaml     # = 现有 nav2_params.yaml（Omni）
    ├── maps/
    └── launch/
        ├── slam.launch.py         # bringup + slam_toolbox
        └── navigation.launch.py   # bringup + include nav2_bringup/bringup_launch.py
```

> 现有 `helios_nav` 的 3 个节点和两个 yaml **直接迁移**进来，几乎不用重写，只调整 odom_publisher 的 TF 行为（见下）。

### 4.3 关键改动点（相对现有 helios_nav）

1. **odom 链路改成 linorobot2 式的 EKF 链**
   - `odom_publisher`：把输出话题改为 `odom/unfiltered`，**关闭自身 TF 发布**（`publish_tf=false`）。
   - 启动 `robot_localization` 的 `ekf_node`，读 `odom/unfiltered`（有 IMU 再加 `imu/data`），输出 `/odom` 和 `odom→base_footprint` TF。
   - 好处：有/无 IMU 都用同一套；后续加 IMU 零改动。

2. **navigation.launch.py 改成 include 官方 nav2_bringup**
   - 删掉手写的一串 nav2 节点 + 手搓 lifecycle_manager。
   - 改为 `IncludeLaunchDescription(nav2_bringup/bringup_launch.py)` + `params_file=navigation.yaml`，更标准、生命周期统一。

3. **坐标系统一**
   - linorobot2 用 `base_footprint`；现有 helios 配置部分用 `base_link`。**统一成 `base_footprint` 为运动基准**（URDF 里 `base_footprint→base_link`），同步改 amcl/slam/costmap 的 `base_frame`。

4. **保留 Helios 专属适配**
   - Omni 参数（amcl `OmniMotionModel`、MPPI `motion_model: Omni`、velocity_smoother 三轴限速）。
   - 双雷达：slam/amcl 用合并后的 `/scan`；**costmap 直接吃两路原始 scan**（多 `observation_sources`，避免合并损失）。
   - footprint 按含臂安全包络配置。

### 4.4 数据流（改造后）

```
真机：
  sr_amr_control SDK
    ├─ front/rear scan ─▶ laser_preprocess ─▶ /scan ─▶ slam_toolbox / amcl
    ├─ system_state    ─▶ odom_publisher  ─▶ odom/unfiltered ─▶ EKF ─▶ /odom + odom→base_footprint
    └─ remote_cmd_vel  ◀─ cmd_vel_relay   ◀─ /cmd_vel ◀─ Nav2
  URDF(robot_state_publisher) ─▶ base_footprint→base_link→雷达 TF
  amcl ─▶ map→odom
  → 完整 TF：map→odom→base_footprint→base_link→雷达

仿真（helios_gazebo，验证算法用）：
  Gazebo planar_move ─▶ /odom + TF；ray 插件 ─▶ scan；订阅 /cmd_vel
  上层 slam_toolbox/amcl/Nav2 与真机完全相同
```

---

## 五、落地步骤（按顺序，建议先仿真后真机）

### 阶段 0：环境与依赖（一次性）
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan
# 可选避障增强：ros-humble-spatio-temporal-voxel-layer
```

### 阶段 1：建包骨架（基于 linorobot2 改）
- 按 §4.2 建四个包，迁移现有 3 个桥接节点 + 两个 yaml。
- `odom_publisher` 改 `odom/unfiltered` + `publish_tf=false`。
- 写 `ekf.yaml`（参照 linorobot2_base/config/ekf.yaml，world_frame=odom、two_d_mode=true）。

### 阶段 2：仿真闭环（不依赖真机，最快验证）
```bash
cd ~/helios_nav_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch helios_description display.launch.py     # 验 URDF/TF
ros2 launch helios_gazebo sim_slam.launch.py         # 建图，遥控走一圈
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_sim
ros2 launch helios_gazebo sim_nav.launch.py          # Nav2 闭环，RViz 发 Goal
ros2 run tf2_tools view_frames                       # 验 map→odom→base_footprint→base_link→laser
```

### 阶段 3：真机最小闭环
```bash
# 1) 启动 SDK（含雷达）
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=192.168.71.50 lidar:=true
# 2) 填真实雷达外参到 static_tf / URDF；先用单雷达
ros2 launch helios_navigation slam.launch.py          # 建图
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_map
# 3) 定位 + 导航
ros2 launch helios_navigation navigation.launch.py map:=/home/admin/maps/helios_map.yaml
```
验证：`ros2 topic hz /scan`、`ros2 topic echo /odom --once`、TF 树完整、RViz 2D Pose Estimate + Nav2 Goal 能闭环。

### 阶段 4：双雷达 + 增强
- 合并器换成成熟方案（如 `2D_Scan_Merger_ROS2`）喂 slam/amcl；costmap 双路原始 scan。
- 拿到 IMU → EKF 加 `imu0`。
- TOF 可用 → `pointcloud_to_laserscan` 或 STVL/nvblox 接 costmap。

### 阶段 5：平台对接
- 封装任务下发 / 状态回传 / 地图管理 API，接入公司平台。

---

## 六、验收检查清单

- [ ] `ros2 topic hz /scan` 稳定（≈5Hz 或合并后频率）
- [ ] `/scan` 中 `101.0` 已变 `inf`（无虚假远距离障碍）
- [ ] `ros2 topic echo /odom --once` 有数据，运动时位姿变化合理
- [ ] TF 树完整：`map→odom→base_footprint→base_link→{雷达}`，无断链
- [ ] amcl 给初始位姿后定位稳定，不漂
- [ ] Nav2 Goal 能规划 + 避障 + 到点（初期限速 0.2–0.4 m/s）
- [ ] 急停/远程控制开关行为正确
- [ ] footprint 覆盖含臂安全包络，导航时双臂已回收

---

## 七、风险与注意事项

1. **odom/外参是硬前置**：无可用 odom + TF，amcl 无法运行；务必先解决里程计来源与雷达外参。
2. **雷达 5Hz/1° 偏低**：移动建图/避障需实测，必要时降速。
3. **轮臂特殊性**：导航时双臂回收；footprint 按最大包络而非仅底盘；2D 激光有盲区，建议接 TOF。
4. **不要同时维护多份 linorobot2**：仓库内有顶层 `linorobot2/`、`lios/_ref/linorobot2_humble/`、`lios/_ref/linorobot2/`，**以 Humble 那份为基准**，其余仅作参考。
5. **桥接层是 Helios 专属**：linorobot2 的 micro-ROS 底层替代不了它，这部分必须自维护。
