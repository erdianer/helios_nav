# Helios 轮臂机器人底盘 2D 导航方案总结

> 目标：脱离厂家 MATRIX，用开源算法栈实现 Helios 底盘的 2D 建图与自主导航，接入公司统一平台。
> 路线：**先在 Gazebo + RViz 跑通，再上真机**；缺什么补什么，关键参数后续找厂家要。
> 最后更新：2026-06-26

---

## 1. 现在的方案（算法栈）

**一句话**：2D 激光 SLAM 建图 + AMCL 定位 + Nav2 全向导航（MPPI 局部控制），中间用自写的 3 个 Python 节点把厂家 SDK / Gazebo 桥接到标准 ROS 接口。

### 数据流

```
底层（仿真 Gazebo 或 真机 sr_amr_control SDK）
        │ 原始话题
        ▼
自写桥接层（helios_nav）
  laser_preprocess  → /scan（过滤 101.0，可选合并双雷达）
  odom_publisher    → /odom + TF（真机用；仿真由 Gazebo 提供）
  cmd_vel_relay     → 底盘 remote_control_cmd_vel（真机用）
        │ 标准接口 /scan /odom /cmd_vel /TF
        ▼
开源导航栈（apt 安装，配置在 helios_nav/config）
  建图 slam_toolbox ｜ 定位 map_server+amcl ｜ 导航 Nav2 全栈
```



### 各模块算法


| 阶段   | 组件                         | 说明                         |
| ---- | -------------------------- | -------------------------- |
| 建图   | **SLAM Toolbox**（async）    | 在线 2D 激光 SLAM，扫描匹配 + 位姿图优化 |
| 定位   | **map_server + AMCL**      | 粒子滤波，运动模型 **Omni 全向**      |
| 全局规划 | **Nav2 Smac Planner 2D**   | 静态地图上规划路径                  |
| 局部控制 | **Nav2 MPPI Controller**   | 运动模型 **Omni**，支持 vx/vy/wz  |
| 路径平滑 | Simple Smoother            |                            |
| 恢复行为 | Behavior Server            | spin / backup / wait       |
| 决策编排 | BT Navigator               | 行为树                        |
| 速度平滑 | Velocity Smoother          | 输出 /cmd_vel 限速平滑           |
| 代价地图 | Obstacle + Inflation Layer | 用 /scan 检测障碍               |




### 仿真 vs 真机：算法相同，只换底层


|              | 仿真（helios_gazebo）              | 真机（helios_nav + SDK）      |
| ------------ | ------------------------------ | ------------------------- |
| `/scan`      | Gazebo 雷达插件 + laser_preprocess | SDK 雷达 + laser_preprocess |
| `/odom` + TF | Gazebo planar_move 插件          | odom_publisher 速度积分       |
| `/cmd_vel`   | Gazebo 订阅                      | cmd_vel_relay → SDK       |
| 建图/定位/导航     | **同一套 slam_toolbox/amcl/Nav2** | **同一套**                   |


---



## 2. 工程结构

```
helios_nav_ws/src/
├── helios_description/          # 机器人模型（仿真/真机共用）
│   ├── urdf/helios.urdf.xacro   # 全向底盘 + 双雷达 + 4轮 + 4个TOF占位frame
│   ├── launch/display.launch.py # 只看模型（不用 Gazebo）
│   └── rviz/display.rviz
├── helios_gazebo/               # 仿真
│   ├── worlds/minimal.world     # 8x8 房间 + 障碍柱
│   ├── launch/sim.launch.py     # Gazebo + 机器人 + 雷达 + RViz
│   ├── launch/sim_slam.launch.py# 仿真 + 建图
│   ├── launch/sim_nav.launch.py # 仿真 + 建图 + Nav2 导航
│   └── README.md
└── helios_nav/                  # 桥接节点 + 导航配置（真机主力）
    ├── helios_nav/laser_preprocess.py  # 101.0→inf，双雷达合并
    ├── helios_nav/odom_publisher.py    # system_state 速度积分→/odom
    ├── helios_nav/cmd_vel_relay.py     # /cmd_vel→SDK
    ├── config/slam_toolbox.yaml
    ├── config/nav2_params.yaml         # amcl Omni + MPPI Omni + costmap
    ├── config/ekf.yaml                 # 预留（有 IMU 再启用）
    └── launch/{bringup,mapping,localization,navigation}.launch.py
```

---



## 3. 使用方法（Linux + ROS2 Humble）



### 3.1 装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro ros-humble-rviz2 \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard ros-humble-robot-localization
```



### 3.2 编译

```bash
cd ~/helios_nav_ws
colcon build --symlink-install
source install/setup.bash      # zsh 用 setup.zsh
```



### 3.3 仿真四步（由简到全，建议按顺序）

```bash
# ① 只看模型，验证 URDF/TF（最快，不用 Gazebo）
ros2 launch helios_description display.launch.py

# ② 仿真 + 遥控
ros2 launch helios_gazebo sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ③ 仿真 + 建图
ros2 launch helios_gazebo sim_slam.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_sim   # 建好后存图

# ④ 仿真 + 自主导航（RViz 点 Nav2 Goal）
ros2 launch helios_gazebo sim_nav.launch.py
```



### 3.4 验证命令

```bash
ros2 topic hz /scan          # 雷达频率
ros2 topic echo /odom --once # 里程计
ros2 run tf2_tools view_frames  # TF 树：map→odom→base_footprint→base_link→laser
```



### 3.5 真机流程（有地图后）

```bash
# 先启动厂家 SDK（含雷达）
ros2 launch sr_amr_control amr_control.launch.py connect_ip:=192.168.71.50 lidar:=true
# 建图
ros2 launch helios_nav mapping.launch.py
# 定位 + 导航
ros2 launch helios_nav navigation.launch.py map:=/home/admin/maps/helios_map.yaml
```

---



## 4. 已确认的接口（厂家 SDK / 手册）


| 能力               | 来源                                | 状态                              |
| ---------------- | --------------------------------- | ------------------------------- |
| 双 2D 激光 `/scan`  | `/sr_amr_control/front|rear/scan` | ✅ 360°/1°/0.1~25m/5Hz，无效值 101.0 |
| 速度（合成 odom）      | `system_state` vx/vy/wz           | ✅ 3.33Hz                        |
| 速度控制             | `remote_control_cmd_vel` (Twist)  | ✅                               |
| 急停               | `emergency_stop` / `release`      | ✅                               |
| MATRIX 现成位姿      | `system_state.current_pose`       | ✅ map 坐标，可作真值参照                 |
| 双激光雷达（硬件）        | 对角安装，前后各一                         | ✅                               |
| 4 个 TOF 避障相机（硬件） | 底盘前后左右，地面 5cm~1m，检测低矮障碍           | ⚠️ 硬件有，ROS 接口未知                 |


---



## 5. 还需要什么（按优先级，建议尽快问厂家）


| 缺口                                     | 影响           | 现状/绕过                              |
| -------------------------------------- | ------------ | ---------------------------------- |
| **雷达真实安装外参** (base_link→前/后雷达 xyz/yaw) | 建图质量、双雷达合并   | URDF 用拟合值占位，待替换                    |
| **底盘真实尺寸 / footprint**                 | costmap、碰撞   | 用手册 0.68×0.64 + 估计高度               |
| **全向速度/加速度上限** vx/vy/wz max            | Nav2 限速、避免超限 | 暂填保守值（vx 0.4, vy 0.4, wz 1.0）      |
| **能否直接给 /odom**                        | 里程计精度        | 现靠 system_state 速度积分（3.33Hz，有漂移）   |
| **能否给 IMU**                            | 定位稳定性        | 无；ekf.yaml 已预留，有则启用 EKF 融合         |
| **4 个 TOF 能否给 ROS2 接口**（点云/深度）+ 外参     | 低矮障碍避障       | 待确认；可做 pointcloud_to_laserscan 补盲区 |




### 里程计专项说明

- 现状：无现成 `/odom`，靠 `system_state` 速度积分生成，**频率低(3.33Hz)+会漂移+无 IMU 纠偏**。
- 能跑：靠 slam_toolbox / amcl 的激光扫描匹配纠偏，可用但精度打折。
- 改善：拿到真 odom 或 IMU 后明显提升。

---



## 6. 可参考的开源方案


| 项目                                                                                              | 借鉴点                                                    | 能否直接用      |
| ----------------------------------------------------------------------------------------------- | ------------------------------------------------------ | ---------- |
| [Trkkhrmn/ros2_amr_mecanumbot](https://github.com/Trkkhrmn/ros2_amr_mecanumbot)                 | **多包结构 + Gazebo 全向仿真 + 硬件桥接**（Humble，已 fork 到 `_ref/`） | 骨架可改，硬件层需换 |
| [adityakamath/lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2)                         | **全向 nav2_params + EKF**（3轮全向，只有 LiDAR，最像）             | 参数可抄       |
| [nav2_bringup (humble)](https://github.com/ros-navigation/navigation2/tree/humble/nav2_bringup) | 官方 launch/参数范式                                         | 权威参考       |


> 当前 `helios_description` / `helios_gazebo` 即参考 `ros2_amr_mecanumbot` 结构 + `lekiwi` 全向参数搭建。

---



## 7. 待办与演进

- [ ] 仿真跑通建图→定位→导航全流程（切系统后第一步）
- [ ] 找厂家补齐第 5 节缺口参数，替换 URDF / nav2_params 占位值
- [ ] 真机跑通同一套上层（只换底层 SDK 桥接）
- [ ] （可选）拿到 IMU → 启用 robot_localization EKF
- [ ] （可选）TOF 可用 → 接入 Nav2 代价地图补激光盲区
- [ ] （后期）对接公司平台：任务下发、状态回传、视频流

---



## 8. 环境信息速查


| 项         | 值                                                    |
| --------- | ---------------------------------------------------- |
| Orin 算力机  | `admin@192.168.71.51`                                |
| 底盘/MATRIX | `192.168.71.50`                                      |
| ROS 版本    | Humble（Orin 默认 zsh，用 setup.zsh）                      |
| 完整 SDK    | `/home/admin/agvsdk/standard_robots_amr_ros2-v1.3.0` |
| 测试地图      | `AB_0619`（站点 A=1, B=2）                               |
| 注意        | 项目里 luoshi 精简版 SDK 无雷达模块，测雷达需完整 SDK v1.3.0           |


```

```

