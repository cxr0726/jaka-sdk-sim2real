# KPI MuJoCo Sim Demo 用户指导手册

本文说明 `sim/demo` 的主程序工作逻辑，以及客户如何替换或接入自己的 LOCO 策略。

## 1. 目录角色

```text
sim/
  sdk/
    include/kpi_mujoco_sim/      # MuJoCo SDK 公开头文件
    lib/release/                 # libkpi_mujoco_sim.so
    third_party/                 # MuJoCo / GLFW
    assets/khan_mini/            # XML / meshes

  demo/
    config/KpiConfig.yaml        # LOCO 策略和控制器基础配置
    config/KpiConfig.sim.yaml    # MuJoCo 仿真专用 PD 和 base 初始位姿
    config/policy_loco/          # LOCO ONNX 策略
    src/KpiSimMain.cpp           # demo 入口
    src/FSMInit.cpp              # 初始化状态
    src/FSMIdle.cpp              # 使能、回零、进入 LOCO
    src/FSMLoco.cpp              # LOCO 策略推理和关节目标输出
    src/FSMFault.cpp             # 故障/复位
    scripts/run_demo.sh          # Release 运行脚本
    scripts/build_debug.sh       # VSCode/gdb Debug 构建脚本
```

## 2. main 主流程

入口文件是 `src/KpiSimMain.cpp`。主流程如下：

1. 解析命令行参数。
2. 初始化 ROS 2，但主仿真反馈来自 MuJoCo SDK 后端。
3. 调用 `RLController::KpiConfigInit()` 读取控制器配置和加载 ONNX 策略。
4. 根据配置创建 `MujocoRobotBackend`：
   - XML 路径来自 `--xml`
   - joint names / init target / PD 来自 demo 生成的运行时 YAML
   - base 初始位姿来自 demo 生成的运行时 YAML
5. 创建 `RobotIntf`，并把 MuJoCo backend attach 进去。
6. 进入 `FSM_INIT`。
7. 主循环中不断：
   - 读取脚本手柄、文件手柄或 ROS `/joy`
   - 写入 MuJoCo backend 的 joy state
   - 调用 `RLController::FSMUpdate()`
   - MuJoCo 物理步进 `control_decimation` 次
   - 可选渲染 viewer

简化伪代码：

```cpp
KpiConfigInit(config);
backend = MujocoRobotBackend(xml, config);
mRobot = RobotIntf();
mRobot->attach_backend(backend);
FSMInitEnter(FSM_INIT);

while (running) {
    joy = read_script_or_ros_joy();
    backend->set_joy_state(joy);
    FSMUpdate();
    backend->step_physics(control_decimation);
}
```

## 3. 当前状态机逻辑

当前 demo 只保留最小链路：

```text
FSM_INIT -> FSM_IDLE -> FSM_LOCO -> FSM_FAULT
```

### FSM_INIT

文件：`src/FSMInit.cpp`

职责：

- 等待所有关节进入 operational 状态。
- 设置关节角度限制。
- 设置 MIT 模式和 PD/gain。
- 处理 pitch/roll torque map PD。
- 绑定 NLF2 平滑器初始状态。
- 数据准备好后进入 `FSM_IDLE`。

### FSM_IDLE

文件：`src/FSMIdle.cpp`

职责：

- 处理使能/失能。
- 处理回 home / 回 LOCO 初始位姿。
- 作为进入 LOCO 前的缓冲状态。

默认脚本模式下，`--mode loco` 会自动模拟：

```text
BACK       -> do_enable(true)
LB + Y     -> FSM_LOCO
```

客户如果要接自己的上层业务逻辑，建议在这里替换：

- 何时允许 `do_enable(true)`
- 何时允许进入 `FSM_LOCO`
- 是否加入安全检查、姿态检查、跌倒检查、外部模式命令

### FSM_LOCO

文件：`src/FSMLoco.cpp`

职责：

- 从 MuJoCo backend 获取关节状态和 IMU。
- 构造策略 observation。
- 调用 ONNX Runtime 推理。
- 把策略 action 解码为关节位置目标。
- 通过 `mRobot->set_joint_trajectory()` 下发给 MuJoCo backend。

LOCO observation 当前布局：

```text
0..2      angular velocity * ang_vel_scale
3..5      projected gravity
6..8      command: forward / lateral / yaw
9..35     joint position relative to default_angles
36..62    joint velocity scaled
63..89    last action
```

所以当前配置中：

```yaml
num_actions: 27
num_obs: 90
num_frame_stack: 10
```

ONNX 输入维度必须等于：

```text
num_obs * num_frame_stack = 900
```

ONNX 输出维度必须等于：

```text
num_actions = 27
```

### FSM_FAULT

文件：`src/FSMFault.cpp`

职责：

- 故障后关闭使能。
- 等待 reset 命令或 START 按键。
- reset 后回到 `FSM_INIT`。

## 4. 配置文件关系

sim demo 和 real 工程配置相互独立。运行脚本只合并 demo 自己的两个配置：

```text
demo/config/KpiConfig.yaml
demo/config/KpiConfig.sim.yaml
```

输出到：

```text
demo/build/KpiConfig.demo.yaml
```

Debug 构建输出到：

```text
demo/build_debug/KpiConfig.debug.yaml
```

其中 `demo/config/KpiConfig.sim.yaml` 只放 MuJoCo 仿真专用字段，合并规则：

```text
kps     -> kp
kds     -> kd
kps_pr  -> kp_pitch_roll
kds_pr  -> kd_pitch_roll
```

并额外接受：

```text
init_base_pos
init_base_quat
```

因此：

- `demo/config/KpiConfig.yaml` 放策略、关节顺序、obs/action scale、默认姿态等控制器配置。
- `demo/config/KpiConfig.sim.yaml` 放 MuJoCo 仿真专用 PD 和 base 初始位姿。

注意：仿真 PD 与真机 PD 是两套独立参数。MuJoCo 里的模型、接触、积分步长和执行器近似都不同于真机，所以 `demo/config/KpiConfig.sim.yaml` 中的 `kps/kds/kps_pr/kds_pr` 只用于仿真稳定性和策略验证，不能直接作为真机部署 PD。

## 5. 运行方式

Release 运行：

```bash
cd sim/demo
./scripts/run_demo.sh
```

带 viewer：

```bash
./scripts/run_demo.sh --viewer
```

指定速度命令：

```bash
./scripts/run_demo.sh --loco-forward 0.2 --loco-lateral 0.0 --loco-yaw 0.0
```

使用 ROS joy：

第一次使用前安装手柄 ROS 包：

```bash
sudo apt update
sudo apt install ros-humble-joy-linux
```

启动手柄节点：

```bash
source /opt/ros/humble/setup.bash
ros2 run joy_linux joy_linux_node
```

然后在另一个终端运行 demo：

```bash
./scripts/run_demo.sh --ros-joy --ros-joy-topic /joy --viewer
```

Debug：

```bash
./scripts/build_debug.sh
```

或在 VSCode 里选择：

```text
(gdb) Sim Demo LOCO
(gdb) Sim Demo LOCO ROS joy viewer
```

## 6. 客户替换自己的 LOCO 策略

### 6.1 替换 ONNX 文件

把客户策略放到：

```text
sim/demo/config/policy_loco/
```

例如：

```text
sim/demo/config/policy_loco/customer_loco.onnx
```

然后修改 `sim/demo/config/KpiConfig.yaml`：

```yaml
policy_path: "policy_loco/customer_loco.onnx"
```

### 6.2 确认关节顺序

`joint_names` 必须和策略训练/导出时的 action 顺序一致。

如果客户策略的 action 顺序不同，需要二选一：

1. 修改 `joint_names`、`default_angles`、limit、target、PD 数组，使它们全部匹配新顺序。
2. 在 `FSMLoco.cpp` 中增加 action/order remap。

不要只替换 `policy_path` 而不检查 joint order。否则策略输出会打到错误关节。

### 6.3 确认 observation 布局

当前 demo 假设客户策略 observation 和现有 LOCO 一致：

```text
ang_vel(3), projected_gravity(3), command(3),
joint_pos_rel(27), joint_vel_scaled(27), last_action(27)
```

如果客户策略 observation 不一致，需要修改：

```text
src/FSMLoco.cpp
  construct_loco_single_obs()
```

常见修改点：

- 是否使用 projected gravity。
- command 的含义和 scale。
- 是否需要历史帧。
- joint position 是绝对值还是相对 `default_angles`。
- joint velocity scale 是否一致。
- last action 是否需要归一化。

修改后同步更新：

```yaml
num_obs
num_frame_stack
cmd_scale
ang_vel_scale
dof_pos_scale
dof_vel_scale
```

### 6.4 确认 action 解码

当前 action 解码逻辑在：

```text
src/FSMLoco.cpp
  fill_loco_trajectory_from_policy_output()
```

当前公式：

```cpp
target = action * action_scale + default_angles[i];
```

如果客户策略输出的是绝对关节角、速度、力矩或其他格式，需要在这里修改。

修改后必须确认：

- 输出维度等于 `num_actions`。
- 输出单位是 rad 还是归一化值。
- 是否需要 clamp 到 `up_limit/down_limit`。
- 是否需要 pitch/roll 的 `angle_map` 转换。

### 6.5 确认 ONNX 输入输出维度

程序启动时会断言：

```cpp
num_obs * num_frame_stack == onnx_input_dim
num_actions == onnx_output_dim
```

如果启动时在 `KpiConfigInit()` 失败，优先检查：

- `num_obs`
- `num_frame_stack`
- `num_actions`
- ONNX 模型输入输出 shape

## 7. 客户添加自己的上层逻辑

推荐保留 SDK 和 MuJoCo backend 不动，只改 demo 层：

```text
src/FSMIdle.cpp   # 进入 LOCO 前的业务逻辑
src/FSMLoco.cpp   # 策略 observation/action
src/KpiSimMain.cpp # 输入来源、命令行参数、ROS topic
```

常见接入方式：

1. **自定义启动条件**
   - 在 `FSMIdle.cpp` 中替换 `LB + Y -> FSM_LOCO`。
   - 可以改为 APP 命令、ROS topic、服务调用或客户自己的安全状态机。

2. **自定义速度命令**
   - 当前 JOY 模式使用：
     ```text
     axes_L_Y -> forward
     axes_R_X -> lateral
     axes_L_X -> yaw
     ```
   - 如果要接上层速度指令，可以在 `FSMLocoInferOfflineAction()` 中改 `loco_cmd` 来源。

3. **自定义安全退出**
   - 当前 JOY `A` 或 APP `MOT_TYPE_ABORT` 会进入 `FSM_FAULT`。
   - 客户可以在 `fsm_loco_judge()` 中加入姿态、速度、关节限位等条件。

4. **自定义配置字段**
   - 在 `KpiConfig.yaml` 添加字段。
   - 在 `GlobalVars.cpp::KpiConfigInit()` 中读取。
   - 避免把训练配置整包塞进部署配置，只保留运行时必要字段。

## 8. 修改后建议验证

每次替换策略或改 observation/action 后，至少做以下检查：

```bash
cd sim/demo
./scripts/run_demo.sh --duration 0.08
```

确认：

```text
state=8
err=0
```

再带 viewer 观察：

```bash
./scripts/run_demo.sh --viewer --duration 0
```

如果机器人一启动就倒，优先检查：

- `default_angles` 是否和训练一致。
- `joint_names` 是否和 action 顺序一致。
- `action_scale` 是否和训练导出一致。
- PD 是否过软或过硬。
- `init_base_pos/init_base_quat` 是否合理。
- observation 中 IMU、关节角、速度的坐标系和 scale 是否一致。
