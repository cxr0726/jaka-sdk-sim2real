# 与 `legged_waist_robot_intf` 集成

[English](robot_intf_integration.md)

## 用途与边界

本项目不直接与电机驱动器通信。`KpiDeployReal` 链接
`liblegged_waist_robot_intf.so`，并创建一个
`legged_waist_controller::RobotIntf` 对象。`RobotIntf` 内部拥有一个 ROS 2
节点、订阅器、发布器、服务客户端，以及默认启用的 spin 线程。它负责在 FSM 使用的
同步 C++ API 与运行中的 `legged_waist_controller` 所提供的 ROS 接口之间进行转换。

该集成分为四层：

```text
KPI FSM 和 ONNX 策略
        |
        | RobotIntf C++ 调用和缓存快照
        v
liblegged_waist_robot_intf.so（内部 ROS 2 节点/spin 线程）
        |
        | 通过 DDS 传输的 ROS 2 话题和服务
        v
legged_waist_controller -> EtherCAT/驱动器和传感器
```

随项目提供的公共 API 位于
`ext_libs/legged_waist_controller/include/legged_waist_controller/usr_intf/robot_intf.h`。
`include/RLController.hpp` 包含了它的转发头文件 `robot_intf.hpp`。

## 构建时连接

`CMakeLists.txt` 添加 RobotIntf 的头文件目录，并链接
`legged_waist_robot_intf`。共享对象随项目置于 `runtime_lib` 中；本仓库不包含
RobotIntf 的源代码实现。因此，头文件和共享对象必须保持版本兼容。运行时，该共享对象
还需要随附的 `legged_waist_controller` 和 `voice_player` ROS 类型支持库。

`KPI_BUILD_REAL` CMake 选项控制是否创建 `KpiDeployReal`，默认为 `ON`。

## 对象所有权与生命周期

进程级对象声明为 `RLController::mRobot`，由 `std::unique_ptr` 管理。

1. `main()` 初始化 ROS 上下文，并禁用基于信号的自动关闭。
2. 加载 `config/KpiConfig.yaml`。
3. 构造 `RobotIntf::Options`，启用 Joy 订阅，然后构造 `RobotIntf`。
4. `Options::auto_start_spin` 默认为 `true`，因此构造对象时会启动接口内部的执行器线程。
   如果将该选项改为 `false`，本项目会显式调用 `start()`。
5. 20 ms 周期的 FSM 线程读取 RobotIntf 快照并发送命令。
6. 收到 SIGINT 或 SIGTERM 后，FSM 线程和日志线程停止并完成 join。
7. 进程请求执行 `do_enable(false)`，等待 500 ms，调用 `RobotIntf::stop()`，
   关闭 ROS，最后销毁该对象。

此顺序非常重要：必须在 ROS 上下文仍然有效时停止 RobotIntf。500 ms 延时旨在为禁用
请求到达控制器预留时间，并不代表已经检查了确认响应。

## ROS 运行时配置

除显式启用 Joy 外，本项目目前使用默认的 `RobotIntf::Options`。相关默认值如下：

| 选项 | 本项目使用的默认值 | 作用 |
| --- | --- | --- |
| `controller_node_name` | `legged_waist_controller` | 目标控制器参数节点 |
| `io_prefix` | 空 | 将控制器 I/O 放在根命名空间 |
| `node_base_name` | `legged_waist_robot_intf` | RobotIntf 内部节点名称 |
| `auto_start_spin` | `true` | 在接口内部运行执行器 |
| `service_timeout` | 3000 ms | 等待服务的默认超时时间 |
| `imu_topic` | `/imu/data` | 由 RobotIntf 缓存的 IMU 输入 |
| `joy_topic` | `/joy` | 由 RobotIntf 缓存的手柄输入 |
| `joy_timeout` | 500 ms | 超过此时间后将过期的手柄数据标记为离线 |
| `control_source` | `JOY` | 初始命令源 |
| `loco_move_topic` | `/loco_move` | 可选的应用层运动输入 |
| `pi_state_topic` | `/PiState` | 发布当前控制状态 |

RobotIntf 还会使用默认配置创建一些可选服务，包括 `set_robot_mot_type`、`go_home`、
控制源服务、录制服务和基础配置服务。未注明为完整话题名称的名称会与 `io_prefix`
组合。完整列表请参阅随项目提供的头文件中的 `RobotIntf::Options`。

`startup.sh` 默认使用 Fast DDS，并设置 `ROS_LOCALHOST_ONLY=1`。控制器必须使用兼容的
RMW 实现、相同的 `ROS_DOMAIN_ID`，且 DDS 网络必须可达。如果控制器位于另一台主机上，
请设置 `ROS_LOCALHOST_ONLY=0`。

## 数据与命令流

RobotIntf 的内部执行器会更新线程安全的缓存状态。FSM 的 20 ms 周期线程轮询这些快照：

| RobotIntf API | 本项目中的用途 |
| --- | --- |
| `get_joint_status()` | 获取位置命令/反馈、速度、使能/故障状态和 EtherCAT OP 就绪状态 |
| `has_imu()` / `get_imu()` | 启动就绪检查、策略观测值和 CSV 日志记录 |
| `get_control_source()` | 判断是否应由 Joy 输入驱动状态变化 |
| `get_joy_robot_state()` | 使能/禁用、状态转换、复位和速度命令 |
| `set_joint_trajectory()` | 发布来自空闲规划器或 ONNX 策略的有序位置目标 |
| `do_enable()` | 在 IDLE 中使能，以及在 IDLE、FAULT 和关闭过程中禁用 |
| `clear_error()` | 在 FAULT 状态请求复位驱动器故障 |
| 增益/限位/模式 API | 在 INIT 到 IDLE 转换期间配置驱动器 |
| `set_cur_control_stat()` | 通过 RobotIntf/PiState 发布 FSM 状态 |

`config/KpiConfig.yaml:joint_names` 中的顺序是关节反馈、ONNX 动作向量、增益、限位和
输出轨迹位置之间的约定。所有对应数组都必须严格按照该顺序包含 `num_actions` 个元素。

## FSM 交互

### INIT

INIT 为每个已配置关节调用 `get_joint_status()`，直到所有关节均报告
`ethercat_operational=true`。它还会等待 `has_imu()`。进入 IDLE 前，INIT 会配置
关节角度限位、选择 MIT 运行模式、写入 MIT 增益、配置 PD 行为，并使用反馈值初始化
本地轨迹滤波器。

尽管当前代码在逐关节增益循环中调用 `set_joint_operation_mode()`，该函数实际会作用于
所有关节。

### IDLE

IDLE 读取选定的控制源和最新 Joy 快照。手柄 BACK 键请求执行 `do_enable(true)`，
A 键请求执行 `do_enable(false)`。当所有关节均报告已使能后，对十字键目标值进行滤波，
并使用 `set_joint_trajectory(joint_names, point)` 发送。LB+Y 进入 LOCO。

### LOCO

LOCO 读取关节和 IMU 快照，将 Joy 摇杆轴转换为归一化的运动命令，构造策略观测值，
运行 ONNX 模型，将动作限制在配置的范围内，并每 20 ms 发送一个有序轨迹点。任一关节
变为未使能或发生故障，都会触发向 FAULT 状态转换。

### FAULT

进入 FAULT 后会立即请求执行 `do_enable(false)`。关节故障仍存在时，按 START 会调用
`clear_error()`；故障清除后，按 START 会使 FSM 返回 INIT，从而重新进行就绪检查。

## 返回值与过期数据

大多数 RobotIntf 命令会返回整数状态码；`get_joint_status()` 能够区分未知关节和未进入
运行状态的关节。当前控制代码通常不检查这些返回值，而是观察后续缓存的关节状态。
对于需要立即确认的更改，应显式检查并处理 API 返回值。

在收到 IMU 样本之前，`get_imu()` 会返回 `false`。当 Joy 数据过期时间超过
`joy_timeout` 时，`get_joy_robot_state()` 会返回 `false` 并清空其输出。添加使用
过期值可能不安全的控制逻辑时，调用方应使用该布尔返回值。

## 添加其他 RobotIntf 调用

扩展此集成时：

1. 在构造对象之前，通过 `RobotIntf::Options` 配置名称和可选功能。
2. 保持只使用一个共享 RobotIntf 实例；不要仅为了访问另一个 API 而添加第二个内部
   ROS 节点。
3. 轨迹向量应使用配置的关节名称顺序。
4. 尽可能避免在 20 ms 周期路径中进行阻塞式服务调用；应在状态转换期间执行配置，
   或使用异步 API。
5. 处理返回码，并为缺失或过期的数据定义安全状态。
6. 在调用 `rclcpp::shutdown()` 前停止接口所拥有的线程。

有关权威的函数签名和选项默认值，请查阅随项目提供的 `robot_intf.h`；线路级实现细节
仍封装在随附的共享库中。
