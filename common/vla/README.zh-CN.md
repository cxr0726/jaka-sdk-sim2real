# JAKA SuperZmq 的 VLA 参考缓冲

本目录的 `vla_pose_v1` 后端供实机和 MuJoCo 的 `FSMMimicJakaMiniSuperZmq` 共用。训练参考、VLA 积分姿态和 tracker 参考都使用 `base_link`。pi0.5 一次返回 50×33 的动作块。本地 OpenHLM 客户端默认执行其中 25 步，再请求下一块。客户端每步先发布当前参考姿态，再以 1/30 秒积分下一姿态。低层以这些运动参考构造 tracker 观测。

运行入口是实机或仿真的 `FSMMimicJakaMiniSuperZmq`。两端的 CMake 通过 `KPI_VLA_DIR` 提供头文件路径，将 [RealtimeMotionBufferVla.cpp](RealtimeMotionBufferVla.cpp) 与 [VlaPosePacket.cpp](VlaPosePacket.cpp) 作为源文件编入可执行目标。前者管理订阅、缓冲与控制 tick 采样，后者解码和校验二进制协议。参见[实机构建](../../real/CMakeLists.txt)与[仿真构建](../../sim/demo/CMakeLists.txt)。

## 与原型及 OpenHLM 的对应关系

| 问题 | 对跟踪的影响 | 当前实现 |
|---|---|---|
| 原型的 JSON 分支和遥操作接收器都把 JAKA 序转为 Isaac 序；二进制分支约定已收到 Isaac 序并逐槽复制。现有 OpenHLM 发布器实际发的是腿在前的 JAKA 序 | 例如 wire 的槽 1 是左髋 roll，tracker buffer 的槽 1 是右髋 pitch。数组长度和数值有限性都发现不了串轴 | 解包后重排一次到原有 buffer 顺序，FSM 继续用按模型关节名建立的 `policy_from_buffer` |
| 旧客户端以腰部的 0.83 m 为初始高度，原型二进制消息未标 anchor | 低层命令直接使用五个未来点的绝对 z，初始误差会保留在积分轨迹里。关节向量中腰部那一维仍是正常的被控关节 | 发送端显式给定起始 base 世界位置，每包写明 `anchor_body=base_link` 和 `joint_order=jaka`。Parquet 回放可从来源明确的 action[33:40] 读取第一帧位置和 yaw |
| 原型以到达时间判断 50 ms 间隙，超过阈值只推进一个名义周期 | 已有 i 帧，漏掉一个双帧包后，从下一包补到 i+1、i+2。原型可能把 i+1 排在与 i 相同的时刻。网络抖动也会改变参考速度 | 每个相邻 frame_index 对应精确的 1/30 秒，`steady_clock` 负责接收新鲜度和控制端播放推进 |
| 启动时只收到一个包即 ready，目标时刻被 clamp 到已有帧 | tracker 可能拿到缺乏前视的五帧并过早建立航向对齐 | 连续参考覆盖完整延迟后一次性返回 ready 和五帧。默认配置从首个可用源帧起需要至少五个连续帧 |
| 只重启客户端时，原型还记得上一条轨迹的最大帧号。它会丢弃新轨迹中小于等于旧最大号的帧，之后可能把另一个空间原点接进轨迹 | 同一个帧号可以来自不同次启动 | `JakaTabletopEnv.__init__()` 为新对象生成一个 UUID。换推理块沿用该对象。SuperZmq 在同一次进入期间拒绝更换 UUID；退出会销毁 buffer，下一次进入可从新客户端的第 0 帧重新起步 |
| 原型长度、dtype、有限性校验不完整，缓冲增长还依赖控制线程持续清理 | 异常输入、长积压可能进入控制窗口 | 先验证整包，再提交新帧。128 帧容量覆盖两个线程的交接。重复包不刷新接收年龄，两端使用一次原子窗口快照 |

上游 SONIC 的 [StreamedMotionMerger](https://github.com/OpenHLM-project/OpenHLM/blob/4a8d4cd0ecd8313bb8dbb10d2afac2c2ec9ed1b4/src/GR00T-WholeBodyControl4OpenHLM/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/streamed_motion_merger.hpp#L125-L205) 按全局帧号合并窗口，更新播放游标偏移，并支持特定条件下的 catch-up。其 [streamed 播放循环](https://github.com/OpenHLM-project/OpenHLM/blob/4a8d4cd0ecd8313bb8dbb10d2afac2c2ec9ed1b4/src/GR00T-WholeBodyControl4OpenHLM/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp#L3381-L3408) 为未来观测预留帧。G1 的关节表和 pose 字段与 Khan 不同，11 帧上限也只出现在指定 encoder 模式。本实现复用帧号与保留前视的思路，使用 Khan 自己的 30→50 Hz 插值、五帧观测和故障保持策略。

## 协议和时序

ZMQ 单条消息由 `pose` 四字节前缀、1280 字节 JSON 头和小端连续数组构成。头声明协议 v1、本实例的 stream_id、base_link 和 jaka。必需数据为 `joint_pos[N,27]`、`body_pos_w[N,3]`、`body_quat_w[N,4]`（wxyz），均为 float32，以及 int64 的 `frame_index[N]`。发送端还提供 `joint_vel[N,27]`，当前 tracker 的参考通道只消费位置与姿态。常规 N=2，每包携带前一帧和当前帧。协议中的 count 表示消息计数，N 由字段形状给出。

默认五个采样点为 `P + [0,20,40,60,80] ms`。令 `L` 为最新连续帧对应的数据时刻，播放上界为 `L − 80 ms − 40 ms`。游标最多推进本次实际单调时间增量，晚到的控制 tick 单步推进上限为两个控制周期。目标位置与关节角线性插值，姿态取最短弧 SLERP。新鲜度按上一次**提供了新参考帧**的包在控制器上的接收时刻计算。120 ms 是数据轴上的保留量，实际端到端延迟还包括推理、网络、采样相位和闭环响应。

实机 50 Hz 线程在 [KpiMain.cpp](../../real/src/KpiMain.cpp) 的 `rt_func()` 先执行 `FSMUpdate()`，再用 `sleep_until` 等到绝对截止时间。如果本轮耗时 3 ms，通常还需等待约 17 ms。SuperZmq 提取窗口时，缓冲在锁内读取单调时钟并减去上次有效采样时刻，测到的差值包括本轮和上轮两次查询之间的工作与调度等待。首次窗口就绪时游标直接定位到上界。

| 状态 | 交给 tracker 的参考 |
|---|---|
| 初次等待或窗口未满 | 配置中的等待姿态。首次得到完整窗口时才对齐其 yaw |
| 正常跟踪 | 同一时刻快照中的五个插值参考 |
| 短暂推理停顿 | 游标可以先前进到上界，随后完整五帧固定。窗口中仍可有相对位移，机器人继续闭环跟踪 |
| 收到互相矛盾的重叠、不可补回的帧号缺口、新会话、缓冲溢出，或新帧超时 | 五个目标全部等于最近一次交给 tracker 的第一个参考，未来相对位移为零。若尚未跟踪则保持等待姿态。FSM 报告 `stream_hold`，操作员可正常退出并重新进入 |

默认超时为 1 秒，是可配置的运维选择。短暂停顿与正式停止在“没有后续包”期间无法区分。**干净停发也不会自动播放完最后约 120 ms 的保留段。** 对需要到达最后一个参考的离线回放，可以继续按 30 Hz 发送终点静止姿态，直至尾段播放完成。重新进入后要以新的起始姿态核对 base 高度、航向和模型输入。数据流状态只描述参考生成情况，实际平衡、接触和任务成功需单独评估。

## 配置

在实际使用的 `jakaminimimic_superzmq` YAML 段选择后端。仓库默认使用原有遥操作后端。实机示例 YAML 的 tracker 路径目前指向 160k residual，复现下文 180k EMA 时还需选择对应的 ONNX 和部署参数。

实机控制进程在启动时读取其工作目录的 `./config/KpiConfig.yaml`。普通 PICO 遥操作选 `motion_backend: legacy_arrival`，pi0.5 选 `motion_backend: vla_pose_v1`。切换后端时改此项并重启控制进程。只换一条 VLA 演示时，退出 Python 发布客户端和 SuperZmq，复位机器人，再进入同一 FSM 并用新的起始位置启动客户端即可。同一 Python 终端内选择继续下一轮会保留积分位置、yaw 和帧号。该入口本身有 2 秒等待，超过默认 1 秒参考超时，原 FSM 会进入保持。物理复位后应重新创建客户端和 buffer。

```yaml
jakaminimimic_superzmq:
  motion_backend: vla_pose_v1
  vla_source_hz: 30
  vla_stale_timeout_s: 1.0
  dt_s: 0.02
  tolerance_s: 0.04
  future_steps: [0, 1, 2, 3, 4]
```

部署时保留仓库中的 `common/vla` 与 `real` 或 `sim/demo` 的相对路径，两套 CMake 使用同一份参考实现。控制器的 `zmq_connect` 连接运行客户端的 PC 的运动 PUB 地址。客户端通过 `--jaka-state-address` 与 `--jaka-head-image-address` 指向 state 和 RGB 发布端。模型服务使用此次 base_link 训练的 checkpoint 与对应归一化统计。客户端选 `--env jaka_tabletop_pick`，用 `--jaka-initial-anchor-pos X Y Z` 给出起始 base 的世界位置（米）；初始 yaw 可选，默认 0 rad。实机的 StatePublisher 提供实测关节和 base 姿态，body_pos_w 是占位值，无法提供起始高度。回放 episode_0002 的首帧参考 z 为 0.608514 m，实机应使用当次起始参考姿态适用的高度。实机 VLA 模式在进入 FSM 前检查 state 端口可绑定，绑定失败会留在 IDLE 并输出具体端点。采集仓库内的 `thirdparty/OpenHLM/doc/07-jaka-deploy-tutorial.md` 包含服务和客户端命令。打开已有的 policy diagnostic 后，`latest_frame_index`、`valid_receive_age_ns` 和故障时的 `stream_hold_reason` 来自本 tick 的同一次窗口采样。

## 离线验证

在仓库根目录加载 ROS Humble 环境，并确保部署 SDK、ONNX Runtime 与 ROS 依赖可用。以下测试均使用本地数据或替代硬件接口。

```bash
cmake -S sim/demo -B build-sim -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-sim --target KpiMujocoSimDemo VlaPosePacketTest VlaMotionBufferTest LegacyRealtimeMotionBufferTest
ctest --test-dir build-sim -R 'supermimic_(vla|legacy)' --output-on-failure

export LD_LIBRARY_PATH="$PWD/real/runtime_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cmake -S real -B build-real -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DKPI_BUILD_REAL=ON
cmake --build build-real
ctest --test-dir build-real -R real_supermimic --output-on-failure
```

[协议测试](test_vla_pose_packet.cpp)覆盖形状、类型、有限值和整包校验。[窗口测试](test_vla_motion_buffer.cpp)以带编号的关节值核对所有 27 个槽，还检查 30→50 Hz 插值、凑满前视、会话重启和中断恢复。在每 25 步出现 120 ms 停顿的确定性测试中，控制器起始相位取 0–19 ms，并分别模拟完整传输和每次换块漏掉首包。[实机 FSM 夹具](../../real/tests/test_supermimic_fsm.cpp)验证等待与保持期间的跟踪、IMU 故障处理、状态发布和退出清理。
