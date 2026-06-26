# WSL 脚本说明

本目录存放 **WSL2 + ROS 2 Jazzy** 环境下与 MoveIt 2 / pymoveit2 相关的辅助脚本。

所有脚本均应在 **WSL 终端**内运行，不要从 Windows PowerShell 直接调用 `ros2 run`（参数引号会被截断）。

## 前置条件

- WSL2 Ubuntu 24.04，已安装 ROS 2 Jazzy
- 工作空间 `~/ros2_ws` 已按 [`wsl_setup_moveit.sh`](#wsl_setup_moveitsh) 配置
- 建议在 `~/.bashrc` 中加入：

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM=xcb
source ~/ros2_ws/install/local_setup.bash
```

从仓库根目录调用脚本的示例路径：

```bash
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/<脚本名>.sh
```

---

## 按场景使用

### 场景 1：首次配置环境（只需运行一次）

**目标：** 安装依赖、克隆 `panda_gz_moveit2` + `pymoveit2`、打 Jazzy 补丁并编译。

| 脚本 | 说明 |
|------|------|
| [`wsl_setup_moveit.sh`](#wsl_setup_moveitsh) | 一键完成上述全部步骤（需要 `sudo` 密码） |

```bash
bash scripts/wsl_setup_moveit.sh
```

---

### 场景 2：验证 MoveIt 2 机械臂节点能否正常运行

**目标：** 确认 `move_group`、控制器、`pymoveit2` 关节运动链路畅通。这是最基本的冒烟测试。

**步骤：**

1. **终端 1** — 启动 RViz + MoveIt（fake 硬件，无 Gazebo）：

```bash
bash scripts/wsl_launch_fake_control.sh
```

看到 `Stack ready` 且无持续 `FATAL` 即可。

2. **终端 2** — 发送关节空间目标，让机械臂动起来：

```bash
bash scripts/wsl_test_joint_goal.sh
```

**期望结果：** 终端输出 `Moving to {joint_positions: ...}`，RViz 中 Panda 机械臂运动。

| 脚本 | 作用 |
|------|------|
| [`wsl_launch_fake_control.sh`](#wsl_launch_fake_controlsh) | 启动 fake control 栈（含延迟 spawn 控制器，修复 Jazzy 竞态） |
| [`wsl_test_joint_goal.sh`](#wsl_test_joint_goalsh) | 调用 pymoveit2 内置 `ex_joint_goal.py` 做关节目标测试 |

---

### 场景 3：验证末端笛卡尔规划（位姿目标）

**目标：** 给定末端 `(x, y, z)` 与姿态，由 MoveIt 规划并执行。

**前提：** 场景 2 的终端 1 已运行 `wsl_launch_fake_control.sh`。

**终端 2：**

```bash
bash scripts/wsl_test_pose_goal.sh
```

**期望结果：** 输出 `Moving to {position: ...}`，RViz 中末端执行器朝目标位姿运动。

| 脚本 | 作用 |
|------|------|
| [`wsl_test_pose_goal.sh`](#wsl_test_pose_goalsh) | 单次笛卡尔位姿目标（`ex_pose_goal.py`） |

---

### 场景 4：Gazebo 物理仿真下运行 MoveIt

**目标：** 在 Gazebo Harmonic 中加载 Panda，配合 MoveIt 控制。

**终端 1：**

```bash
bash scripts/wsl_launch_gz_control.sh
```

在 Gazebo 窗口点击 **Play**，再于 **终端 2** 运行场景 2 或 3 的测试脚本。

| 脚本 | 作用 |
|------|------|
| [`wsl_launch_gz_control.sh`](#wsl_launch_gz_controlsh) | 启动 `ex_gz_control.launch.py`（Gazebo + RViz + MoveIt） |

---

### 场景 5：避障规划（碰撞物体 + 重规划）

**目标：** 向规划场景添加障碍球体，再尝试末端位姿规划。

**前提：** 终端 1 已运行 `wsl_launch_fake_control.sh`。

**终端 2：**

```bash
bash scripts/wsl_test_collision_planning.sh
```

流程：添加球体 → 规划 EE 位姿 → 尝试清空场景。

**说明：** `panda_gz_moveit2` 的 URDF 缺少 link 碰撞网格，碰撞感知规划可能返回 `FAILURE`；添加障碍物体到场景仍会成功，可在 RViz 中看到。清空场景需 `pip3 install trimesh`。

| 脚本 | 作用 |
|------|------|
| [`wsl_test_collision_planning.sh`](#wsl_test_collision_planningsh) | `ex_collision_primitive.py` + `ex_pose_goal.py` + `ex_clear_planning_scene.py` |

---

### 场景 6：一次性跑完全部末端测试

**目标：** 关节、位姿、夹爪、障碍、带障重规划串联执行。

**前提：** 终端 1 已运行 `wsl_launch_fake_control.sh`。

**终端 2：**

```bash
bash scripts/wsl_run_ee_tests.sh
```

| 脚本 | 作用 |
|------|------|
| [`wsl_run_ee_tests.sh`](#wsl_run_ee_testssh) | 批量运行 pymoveit2 官方示例（5 项测试） |

---

## 脚本速查表

| 脚本 | 场景 | 需要几个终端 | 依赖 launch |
|------|------|-------------|-------------|
| `wsl_setup_moveit.sh` | 首次安装 | 1 | 无 |
| `wsl_launch_fake_control.sh` | 启动 fake 栈 | 1（保持运行） | 自包含 |
| `wsl_launch_gz_control.sh` | 启动 Gazebo 栈 | 1（保持运行） | 自包含 |
| `wsl_test_joint_goal.sh` | 关节运动验证 | 2 | fake 或 gz |
| `wsl_test_pose_goal.sh` | 末端位姿验证 | 2 | fake 或 gz |
| `wsl_test_collision_planning.sh` | 避障规划 | 2 | fake 或 gz |
| `wsl_run_ee_tests.sh` | 全套 EE 测试 | 2 | fake 或 gz |

---

## 各脚本详细说明

### `wsl_setup_moveit.sh`

- **用途：** 新机器或重装后，配置 MoveIt + pymoveit2 工作空间。
- **操作：** `apt` 安装 ROS 包 → 克隆 `panda_gz_moveit2`（jazzy 分支）与 `pymoveit2` → 将 `fake_components` 改为 `mock_components` → `colcon build`。
- **注意：** 需要 `sudo`；`rosdep install` 失败时可忽略，以 `colcon build` 结果为准。

### `wsl_launch_fake_control.sh`

- **用途：** 启动 `ex_fake_control.launch.py`（RViz + move_group + mock ros2_control）。
- **操作：** 清理残留进程 → 后台 launch → 等待 15s → 手动 spawn 三个控制器。
- **日志：** `/tmp/panda_fake_control.log`
- **注意：** 脚本结束后 launch 仍在后台；再次运行前可先 `killall ros2_control_node move_group rviz2`。

### `wsl_launch_gz_control.sh`

- **用途：** 启动 `ex_gz_control.launch.py`（Gazebo + MoveIt）。
- **操作：** 前台运行，Ctrl+C 结束。
- **注意：** 需在 Gazebo 中点击 Play 后，仿真时间才会推进。

### `wsl_test_joint_goal.sh`

- **用途：** 验证 pymoveit2 关节空间规划与执行。
- **底层命令：** `ros2 run pymoveit2 ex_joint_goal.py`
- **典型用途：** 判断 MoveIt 2 自带（经 pymoveit2 调用的）机械臂控制链路是否正常。

### `wsl_test_pose_goal.sh`

- **用途：** 验证末端笛卡尔位姿规划。
- **底层命令：** `ros2 run pymoveit2 ex_pose_goal.py`
- **默认目标：** position `[0.25, 0.0, 1.0]`，四元数 `[0, 0, 0, 1]`。

### `wsl_test_collision_planning.sh`

- **用途：** 练习避障规划流程。
- **步骤：** 添加球体障碍 → EE 位姿规划 → 清空规划场景。
- **可选依赖：** `pip3 install trimesh`（用于 `ex_clear_planning_scene.py`）。

### `wsl_run_ee_tests.sh`

- **用途：** 连续运行 5 项 pymoveit2 官方示例（位姿 ×2、夹爪、障碍、带障位姿）。
- **耗时：** 约 1–2 分钟。

---

## 推荐练习顺序

```
wsl_setup_moveit.sh          # 仅首次
    ↓
wsl_launch_fake_control.sh   # 终端 1
    ↓
wsl_test_joint_goal.sh       # 终端 2：先确认能动
    ↓
wsl_test_pose_goal.sh        # 末端笛卡尔
    ↓
wsl_test_collision_planning.sh
    ↓
wsl_launch_gz_control.sh     # 再上 Gazebo
```

---

## 维护约定

**新增脚本时，必须同步更新本 `README.md`：**

1. 在 [脚本速查表](#脚本速查表) 中增加一行。
2. 在 [按场景使用](#按场景使用) 中归入对应场景（或新建场景小节）。
3. 在 [各脚本详细说明](#各脚本详细说明) 中补充：用途、用法、依赖、注意事项。
4. 若改变推荐流程，更新 [推荐练习顺序](#推荐练习顺序)。

脚本文件头部建议保留一行注释，写明用途与运行环境，例如：

```bash
#!/usr/bin/env bash
# <一句话用途>（在 WSL 内运行；需 fake control 已启动）
```
