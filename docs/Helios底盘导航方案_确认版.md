# Helios 轮臂机器人底盘 2D 导航 —— 确认版方案

> 唯一事实来源（Single Source of Truth）。合并并校准了此前两份草稿：
> 《Helios底盘导航_数据清单与linorobot2改造方案.md》（数据/取舍更准，作基底）
> 《Helios基于linorobot2导航改造实践方案.md》（补充优先级与必改速记）。
> 雷达参数已对齐《雷达测试.md》真机实测。
> 技术栈：slam_toolbox + map_server + amcl + Nav2（全向底盘，局部控制 MPPI Omni）。
> 环境：ROS2 Humble、Orin `192.168.71.51`、底盘 `192.168.71.50`、完整 SDK `sr_amr_control` v1.3.0。
> 最后更新：2026-06-29

---

## 〇、本版校准的关键冲突（先看这条）

| 项 | 两份草稿曾不一致 | 确认结论（本版采用） |
|----|------------------|----------------------|
| 雷达量程 | 25m / 100m | **原始 `range_max=100.0`**（实测）；我们在 `laser_preprocess` 与 nav2 里**裁剪到 25m** 使用（远处低质量点丢弃） |
| 雷达 bin/分辨率 | 1°×?bin | **1° 步进、721 bin、约 5Hz**（实测） |
| 雷达 frame 名 | front/rear_laser_link | **`right_front_laser_link` / `left_behind_laser_link`**（实测 SDK frame） |
| odom 方案 | 直接发TF / 走EKF | **统一走 EKF**：odom_publisher 出 `odom/unfiltered` 且不发 TF，EKF 发 `/odom` + `odom→base_footprint` |
| 工程结构 | fork 整仓 / 原地拆包 | **原地拆 4 包，借鉴 linorobot2 范式但不 fork 其代码**（避免删 70%） |
| 运动基准坐标系 | base_link / base_footprint | **统一 `base_footprint`** |

---

## 一、名词约定：什么是"桥接"

厂家底盘说的是"私有方言"（`sr_amr_control` SDK 的话题/服务），开源导航栈（Nav2/slam_toolbox/amcl）认的是"ROS 标准接口"。**桥接 = 中间一层薄适配节点，不改 SDK、也不改 Nav2，只做话题/坐标系/命令的翻译。**

```
厂家 SDK (sr_amr_control)        桥接层 (自写 Python 节点)          开源导航栈
────────────────────────        ─────────────────────────         ──────────────
front/scan, rear/scan      ──▶  laser_preprocess          ──▶  /scan   ──▶ slam_toolbox / amcl / costmap
system_state(vx,vy,wz)     ──▶  odom_publisher → EKF      ──▶  /odom + TF ─▶ amcl / Nav2
/cmd_vel (Nav2 输出)        ──▶  cmd_vel_relay            ──▶  remote_control_cmd_vel ─▶ 底盘运动
```

---

## 二、底盘导航数据清单（核心参考表）

> 列含义：**导航是否必需** / **现状** / **呈现形式（话题·类型·来源）** / **用途** / **桥接 / 处理方式** / **状态**。

### 2.1 必需数据（缺了导航跑不起来）

| 数据项 | 必需 | 现状 | 呈现形式（话题 / 类型 / 来源） | 用途 | 桥接 / 处理方式 | 状态 |
|---|---|---|---|---|---|---|
| **激光 /scan** | ✅ | 有（双雷达） | `/sr_amr_control/front/scan`、`/sr_amr_control/rear/scan`（`sensor_msgs/LaserScan`，360°/1°/721 bin/约 5Hz，`range_max=100.0`，无效占位 `101.0`） | 建图、定位、代价地图避障 | `laser_preprocess`：`101.0`→`inf`；裁剪到 25m；可选合并双雷达为单帧 `/scan` | 真机已验证 |
| **里程计 /odom + TF odom→base_footprint** | ✅ | 部分（无现成 odom） | 仅 `system_state` 内有车体速度 `linear_velocity_x/y`、`angular_velocity`（约 3.33Hz） | amcl/slam 运动先验；Nav2 局部规划 | `odom_publisher` 速度积分→`odom/unfiltered`（**不发 TF**）→ `robot_localization` EKF → `/odom` + `odom→base_footprint` | 桥接可发布，精度待验证 |
| **速度控制 /cmd_vel** | ✅ | 有（私有话题） | 底盘收 `/sr_amr_control/remote_control_cmd_vel`（`geometry_msgs/Twist`），需先开远程控制服务 | 执行 Nav2 输出运动命令 | `cmd_vel_relay`：`/cmd_vel`→`remote_control_cmd_vel`；启动调 `remote_control_enabled`（`std_srvs/SetBool`） | 桥接已写 |
| **雷达外参 TF base_link→雷达** | ✅ | 无真值（占位） | 静态 TF：`base_link→right_front_laser_link` / `left_behind_laser_link` | 决定 scan 在车体的位置，直接影响建图/合并质量 | `static_tf.launch.py` 占位（前 +0.30 / 后 -0.30）；测试期可让 SDK 把 frame 设成 `map` 跳过 TF | **必须找厂家要真实安装位姿** |
| **TF 完整树** map→odom→base_footprint→base_link→雷达 | ✅ | 部分 | amcl(map→odom)+EKF(odom→base_footprint)+URDF(base→雷达) | 所有定位/导航/可视化的基础 | URDF（robot_state_publisher）+ EKF + amcl 各发一段 | 链路设计已定，需打通 |
| **栅格地图 map** | ✅ | 建图后产出 | `nav_msgs/OccupancyGrid`（`.pgm`+`.yaml`） | 全局定位与全局规划底图 | slam_toolbox 建图→`map_saver_cli` 保存→`map_server` 加载 | 流程已就绪 |
| **初始位姿 /initialpose** | ✅(定位阶段) | 手动 | RViz「2D Pose Estimate」；或参照 `system_state.current_pose` | amcl 初始化 | RViz 工具发布 | 流程已就绪 |
| **机器人外形 footprint / 尺寸** | ✅ | 有手册值 | 底盘 0.68×0.64m（手册）；URDF/nav2 参数配置 | 碰撞检查、膨胀层、贴边通过 | 写入 URDF 与 `navigation.yaml` 的 footprint | 用手册值，待厂家确认精确值与含臂包络 |
| **/clock**（仅仿真） | ✅(仿真) | 有 | Gazebo 提供 | 仿真时间同步 | `use_sim_time:=true` | 就绪 |

### 2.2 可选 / 增强数据（有则更稳，无也能先跑）

| 数据项 | 必需 | 现状 | 呈现形式 | 用途 | 桥接 / 处理方式 | 状态 |
|---|---|---|---|---|---|---|
| **IMU** | ⬜ 可选(强推) | 未确认 | 期望 `sensor_msgs/Imu` | 融合进 EKF，提升旋转/打滑时定位稳定性 | 有则进 `ekf.yaml` 的 `imu0`；可选 `imu_filter_madgwick` 预处理 | 待向厂家确认 |
| **4 路 TOF 深度相机** | ⬜ 可选 | 硬件有，ROS 接口未知 | 期望点云/深度（前后左右，地面 5cm–1m） | 补 2D 激光盲区与低矮障碍 | `pointcloud_to_laserscan`（省事）或 STVL/nvblox 体素层接入 costmap | 待确认 SDK 接口 + 外参 |
| **轮速编码器** | ⬜ 可选 | 无（SDK 不暴露） | — | 高精度 odom | 现用 system_state 速度积分替代 | 缺 |
| **厂家现成位姿（参照）** | ⬜ | 有 | `system_state.current_pose`（map 坐标） | 真值参照、对比自研定位精度 | 仅读取比对，**不用于自研定位**（避免依赖 MATRIX） | 已验证 |
| **急停状态** | ⬜ 建议接 | 有 | `system_state.estop_active`、`emergency_stop`/`release` 服务 | 急停清零速度、上报平台 | `odom_publisher` 已处理 estop 清零；可扩展上报 | 已部分处理 |
| **电池状态** | ⬜ | 手册提供 | `/sr_amr_control/battery_state` | 平台监控、低电返航 | 平台层订阅 | 平台对接阶段用 |

### 2.3 雷达数据已知限制（实践必读）

- **无效值 `101.0`**：表示该方向无回波，必须转 `inf`，否则被当成真实远距离障碍，污染地图与定位。
- **原始量程 0–100m**：远处多为低质量点，`laser_preprocess` 与 nav2 中裁剪到 **25m** 使用。
- **非原始 ring data**：SDK 从 SROS 的 `loc_laser_points`（处理后障碍点）重建 `LaserScan`，不是驱动层原始扫描。
- **只有 LaserScan，无 PointCloud2**。
- **默认关闭**：雷达上报默认 `lidar:=false`，重启节点后需重新开启（或 launch 时 `lidar:=true`）。
- **频率偏低**：约 5Hz、1° 分辨率；移动建图/避障效果需实测，必要时降速。

---

## 三、数据缺口与前置动作（开发前必做）

| 缺口 | 影响 | 现在的绕过 | 要做的事 | 优先级 |
|---|---|---|---|---|
| 能否直接给标准 `/odom` | 里程计精度 | system_state 速度积分（3.33Hz、漂移） | 问厂家有无标准 odom 话题、频率 | **P0** |
| 雷达真实外参（base_link→前/后雷达 xyz/yaw） | 建图质量、双雷达合并 | URDF/static_tf 占位值 | 找厂家要，填入 URDF | **P0** |
| 底盘真实 footprint + 传感器安装高度 | costmap 碰撞 | 手册 0.68×0.64 + 估计 | 找厂家确认 | **P0** |
| 全向 vx/vy/wz 速度+加速度上限 | Nav2 限速、MPPI 采样 | 保守值 vx0.4/vy0.4/wz1.0 | 找厂家实测值 | P1 |
| 能否给 IMU | 定位稳定性 | 无，ekf 预留 | 确认话题/类型/频率，启用 EKF | P1 |
| TOF ROS 接口 + 外参 | 低矮/盲区避障 | 暂不接 | 确认点云/深度话题与安装位姿 | P2 |
| 含机械臂的最大包络 | footprint 安全 | 用底盘 0.68×0.64 | 约定双臂回收姿态 + 实际包络 | P2 |

> 核查命令（Orin 上）：`ros2 topic list -t`、`ros2 run tf2_tools view_frames`、`ros2 topic echo /sr_amr_control/system_state --once`，并查 SDK 的 `README-zh.md`。

---

## 四、算法栈（仿真与真机共用同一套）

| 阶段 | 组件 | Helios 配置 |
|------|------|-------------|
| 建图 | SLAM Toolbox（async） | 在线 2D 激光 SLAM |
| 定位 | map_server + AMCL | **运动模型 OmniMotionModel** |
| 全局规划 | Nav2 Smac Planner 2D | 静态地图规划 |
| 局部控制 | Nav2 MPPI Controller | **motion_model: Omni**，支持 vx/vy/wz |
| 里程计融合 | robot_localization EKF | 轮速(+IMU) → `/odom` + `odom→base_footprint` |
| 代价地图 | Obstacle + Inflation Layer | slam/amcl 用合并 `/scan`；**costmap 吃两路原始 scan** |

> 算法包均 apt 安装（`ros-humble-navigation2`、`ros-humble-slam-toolbox`、`ros-humble-robot-localization`），不进 repo。

---

## 五、基于 linorobot2 的改造方案

### 5.1 为什么"借鉴 linorobot2"而不是 fork 整仓

linorobot2 提供成熟的**上层组织范式**值得复用；但其**底层硬件层（micro-ROS 固件 + linorobot2_base）对 Helios 完全用不上**——Helios 是带 SDK 的成品 AMR，底层用桥接节点替代。

| linorobot2 的部分 | 对 Helios | 处理 |
|---|---|---|
| `linorobot2_base` + micro-ROS 固件 | ❌ 用不上 | 删，用桥接节点替代 |
| `robot_localization` EKF 链路 | ✅ 值得抄 | 保留，odom 输入改成桥接来的 `odom/unfiltered` |
| `navigation.launch.py` → include `nav2_bringup/bringup_launch.py` | ✅ 值得抄 | 保留，换我们的 `navigation.yaml` |
| 分层 launch（bringup/sensors/description/slam/navigation） | ✅ 值得抄 | 保留骨架 |
| 默认差速参数 | ⚠️ 不合适 | 换 Omni（amcl OmniMotionModel + MPPI Omni） |
| 单雷达 | ⚠️ 不够 | 改双雷达（laser 宏实例化两次 + 合并节点） |

**结论：混合方案 = linorobot2 上层骨架（自建同名包，不 fork 代码）+ Helios 桥接层（现有 3 节点）+ 已调好的 Omni 参数。**

> 备选：若确实想"在大项目上改"，也可 `git clone -b humble linorobot2` 后整仓裁剪。但需删除约 70% LIMO/micro-ROS 内容，工时不低于自建。本版推荐自建 4 包。

### 5.2 目标包结构（原地拆包，借鉴 linorobot2 命名）

```
helios_nav_ws/src/
├── helios_description/        # 机器人模型（仿真/真机共用）
│   ├── urdf/helios.urdf.xacro     # 全向底盘 + 双雷达 + 4 TOF 占位 + base_footprint
│   └── launch/description.launch.py   # robot_state_publisher
├── helios_base/              # 对接 SDK 的"底层"（替代 linorobot2_base + micro-ROS）
│   ├── helios_base/laser_preprocess.py   # 101.0→inf，裁剪25m，可选合并
│   ├── helios_base/odom_publisher.py     # system_state→odom/unfiltered（不发 TF）
│   ├── helios_base/cmd_vel_relay.py      # /cmd_vel→remote_control_cmd_vel
│   └── config/ekf.yaml                   # robot_localization 融合配置
├── helios_bringup/           # 启动编排（仿 linorobot2_bringup）
│   └── launch/bringup.launch.py  # description + 桥接节点 + EKF + 静态外参 TF
├── helios_navigation/        # 建图 / 定位 / 导航（仿 linorobot2_navigation）
│   ├── config/slam_toolbox.yaml
│   ├── config/navigation.yaml     # = 现有 nav2_params.yaml（Omni）
│   ├── maps/
│   └── launch/
│       ├── slam.launch.py         # bringup + slam_toolbox
│       └── navigation.launch.py   # bringup + include nav2_bringup/bringup_launch.py
└── helios_gazebo/            # 仿真（保留现有，spawn helios_description）
```

> 现有 `helios_nav` 的 3 个节点和两个 yaml **直接迁移**进来，几乎不用重写，只调整 odom_publisher 的 TF 行为（见 5.3）。
> ⚠️ 拆包前先 `git branch backup-helios_nav` 备份现状，再重构。

### 5.3 关键改动点（相对现有 helios_nav）

1. **odom 链路改成 EKF 链**
   - `odom_publisher`：输出话题改 `odom/unfiltered`，**关闭自身 TF**（`publish_tf=false`）。
   - 启 `robot_localization` 的 `ekf_node`，读 `odom/unfiltered`（有 IMU 再加 `imu/data`），输出 `/odom` 与 `odom→base_footprint`。
   - 好处：有/无 IMU 都用同一套；后续加 IMU 零改动。

2. **navigation.launch.py 改成 include 官方 nav2_bringup**
   - 删掉手写的一串 nav2 节点 + 手搓 lifecycle_manager。
   - 改 `IncludeLaunchDescription(nav2_bringup/bringup_launch.py)` + `params_file=navigation.yaml`，生命周期统一。

3. **坐标系统一 base_footprint**
   - URDF 里 `base_footprint→base_link`；同步改 amcl/slam/costmap 的 `base_frame`、odom_publisher 的 `base_frame`。

4. **保留 Helios 专属适配**
   - Omni 参数：amcl `OmniMotionModel`、MPPI `motion_model: Omni`、velocity_smoother 三轴限速。
   - 双雷达：slam/amcl 用合并 `/scan`；**costmap 用两路原始 scan**（多 `observation_sources`，避免合并损失）。
   - footprint 按含臂安全包络配置。

### 5.4 必改 4 处速记（最容易漏，漏一处全向就跑偏）

1. **AMCL** `robot_model_type: "nav2_amcl::OmniMotionModel"`
2. **Controller** MPPI `motion_model: "Omni"` + `vy_max>0`
3. **base_frame** 全部统一 `base_footprint`
4. **TF 唯一发布者**：odom_publisher（`publish_tf=false`）与 EKF 不重复发 `odom→base_footprint`

### 5.5 数据流（改造后）

```
真机：
  sr_amr_control SDK
    ├─ front/rear scan ─▶ laser_preprocess ─▶ /scan ─▶ slam_toolbox / amcl
    │                                     └▶（两路原始 scan）─▶ costmap observation_sources
    ├─ system_state    ─▶ odom_publisher ─▶ odom/unfiltered ─▶ EKF ─▶ /odom + odom→base_footprint
    └─ remote_cmd_vel  ◀─ cmd_vel_relay  ◀─ /cmd_vel ◀─ Nav2
  URDF(robot_state_publisher) ─▶ base_footprint→base_link→雷达 TF
  amcl ─▶ map→odom
  → 完整 TF：map→odom→base_footprint→base_link→雷达

仿真（helios_gazebo，验证算法用）：
  Gazebo planar_move ─▶ /odom + TF；ray 插件 ─▶ scan；订阅 /cmd_vel
  上层 slam_toolbox/amcl/Nav2 与真机完全相同
```

---

## 六、落地步骤（先仿真后真机）

### 阶段 0：环境与依赖（一次性）
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard
# 可选避障增强：ros-humble-spatio-temporal-voxel-layer
```

### 阶段 1：建包骨架（基于 linorobot2 范式拆包）
- `git branch backup-helios_nav` 备份。
- 按 §5.2 建 4 个包，迁移现有 3 个桥接节点 + 两个 yaml。
- `odom_publisher` 改 `odom/unfiltered` + `publish_tf=false`。
- 写 `ekf.yaml`（参照 linorobot2_base/config/ekf.yaml：`world_frame=odom`、`two_d_mode=true`）。

### 阶段 2：仿真闭环（不依赖真机，最快验证）
```bash
cd ~/helios_nav_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch helios_description description.launch.py    # 验 URDF/TF
ros2 launch helios_gazebo sim_slam.launch.py            # 建图，遥控走一圈
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_sim
ros2 launch helios_gazebo sim_nav.launch.py             # Nav2 闭环，RViz 发 Goal
ros2 run tf2_tools view_frames                          # 验 map→odom→base_footprint→base_link→laser
```

### 阶段 3：真机最小闭环
```bash
# 1) 启动 SDK（含雷达）
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=192.168.71.50 lidar:=true
# 2) 填真实雷达外参到 static_tf / URDF；先用单雷达
ros2 launch helios_navigation slam.launch.py           # 建图
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_map
# 3) 定位 + 导航
ros2 launch helios_navigation navigation.launch.py map:=/home/admin/maps/helios_map.yaml
```

### 阶段 4：双雷达 + 增强
- 合并器换成熟方案（如 `2D_Scan_Merger_ROS2`）喂 slam/amcl；costmap 双路原始 scan。
- 拿到 IMU → EKF 加 `imu0`，odom_publisher 维持 `publish_tf=false`。
- TOF 可用 → `pointcloud_to_laserscan` 或 STVL/nvblox 接 costmap。

### 阶段 5：平台对接
- 封装任务下发 / 状态回传 / 地图管理 API，接入公司平台。

---

## 七、验收检查清单

- [ ] `ros2 topic hz /scan` 稳定（≈5Hz 或合并后频率）
- [ ] `/scan` 中 `101.0` 已变 `inf`（无虚假远距离障碍），且远点裁剪到 25m
- [ ] `ros2 topic echo /odom --once` 有数据，运动时位姿变化合理
- [ ] TF 树完整：`map→odom→base_footprint→base_link→{雷达}`，无断链、无重复发布
- [ ] amcl 给初始位姿后定位稳定，不漂；粒子收敛、激光贴合地图
- [ ] 全向横移：能 vy 横走（非只前进+转）→ 验证 amcl/controller 均为 Omni
- [ ] Nav2 Goal 能规划 + 避障 + 到点（初期限速 0.2–0.4 m/s）
- [ ] 急停/远程控制开关行为正确
- [ ] footprint 覆盖含臂安全包络，导航时双臂已回收

---

## 八、风险与注意事项

1. **odom/外参是硬前置**：无可用 odom + TF，amcl 无法运行；务必先解决里程计来源与雷达外参。
2. **雷达 5Hz/1° 偏低**：移动建图/避障需实测，必要时降速。
3. **轮臂特殊性**：导航时双臂回收；footprint 按最大包络而非仅底盘；2D 激光有盲区，建议接 TOF。
4. **不要同时维护多份 linorobot2**：仓库内有 `lios/_ref/linorobot2_humble/` 等多份，**以 Humble 那份为基准**，其余仅参考。
5. **桥接层是 Helios 专属**：linorobot2 的 micro-ROS 底层替代不了它，必须自维护。
6. **TF 唯一发布者**：EKF 与 odom_publisher 不可同时发 `odom→base_footprint`。

---

## 九、环境信息速查

| 项 | 值 |
|----|----|
| Orin 算力机 | `admin@192.168.71.51`（默认 zsh，用 setup.zsh） |
| 底盘 / MATRIX | `192.168.71.50`（SDK 自动连接） |
| ROS 版本 | Humble |
| 完整 SDK | `/home/admin/agvsdk/standard_robots_amr_ros2-v1.3.0` |
| 仿真器 | Gazebo Classic 11 |
| 工作空间 | `~/helios_nav_ws` |
| 参考代码 | `d:\chx\lios\_ref\linorobot2_humble`、`_ref\agilex_open_class` |
| 关联文档 | 《雷达测试.md》（真机实测）、《底盘2D导航开源方案调研.md》、《Helios底盘2D导航方案总结.md》 |

---

## 十、参考来源

- linorobot2（humble）：https://github.com/linorobot/linorobot2/tree/humble
- 松灵 ROS2 开讲啦配套：https://github.com/agilexrobotics/agilex_open_class
- Nav2 文档：https://docs.nav2.org/
- SLAM Toolbox：https://github.com/SteveMacenski/slam_toolbox
- robot_localization：https://github.com/cra-ros-pkg/robot_localization
