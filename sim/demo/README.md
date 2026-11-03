# KPI MuJoCo Sim SDK Demo

这个 demo 用于验证客户 SDK 包可以接入真机部署控制器的 LOCO 策略逻辑，并通过 MuJoCo SDK 后端提供仿真反馈。

并联踝解算、故障处理及默认 CAD 接触模型见
[ankle and contact deployment](docs/ankle_and_contact_deployment.md)。默认模型为
`sdk/assets/khan_mini/Khan_mini_contact.xml`；旧 XML 保留用于几何回归。

两条 Pico NPZ 与 mjlab 的对照结果、参考高度差异及临时 7 capsule 播放命令见
[OOD deployment comparison](docs/mjlab_ood_comparison.md)。

新 `khan_lafan1_delay_covex` 100k 策略第一次起身踉跄的实测、碰撞消融及临时对照命令见
[new 100k getup comparison](docs/new100k_getup_comparison.md)。

目录关系：

```text
sim/
  sdk/      # 交付 SDK：公开头文件、libkpi_mujoco_sim.so、MuJoCo/GLFW、XML/STL
  demo/     # 策略仿真 demo：控制器源码、策略文件、配置、运行脚本
```

demo 编译时不使用 `sim/*.cpp`，只链接：

```text
../sdk/lib/release/libkpi_mujoco_sim.so
```

## 依赖

客户机器需要：

```text
ROS 2 Humble
Eigen3
yaml-cpp
OpenGL/libGL
```

如果要使用手柄 ROS 输入，还需要安装：

```bash
sudo apt update
sudo apt install ros-humble-joy-linux
```

MuJoCo、GLFW、ONNX Runtime、LOCO 策略文件和 XML/STL 已随 `sim/sdk` 与 `sim/demo` 复制。

## 运行

```bash
cd sim/demo
./scripts/run_demo.sh
```

默认参数：

```text
--mode loco
--duration 5
```

打开 viewer：

```bash
./scripts/run_demo.sh --viewer
```

手动指定模式：

```bash
./scripts/run_demo.sh --mode loco --duration 10 --viewer
```

当前 demo 包含 `init / idle / loco / fault`，以及离线 mimic、JAKA ZMQ 和 fixwrist 遥操作模式。

Khan SuperMimic 导出策略使用独立模式：

```bash
./scripts/run_demo.sh --viewer --duration 0 --mode jakaminimimic_superzmq
```

新模型的关节顺序、base anchor、观测缩放与原 ZMQ 策略分别适配，
详见 [SuperMimic 部署说明](docs/supermimic_deployment.md)。

SuperMimic 本机 NPZ 回放（先启动控制器，再发布动作）：

```bash
./scripts/run_super_npz_demo.sh scripts/newsmallstep.npz
```

也支持带 joint/body names 的 Khan 训练 NPZ。SuperMimic 固定使用 base_link[0]；
普通 ZMQ 和 fixwrist 模式可通过各自的 `reference_body_index` 配置切换 0/13。

更详细的主流程和客户策略接入说明见：

```text
USER_GUIDE.md
```

## 配置生成

运行脚本会把：

```text
demo/config/KpiConfig.yaml
demo/config/KpiConfig.sim.yaml
```

合并成：

```text
demo/build/KpiConfig.demo.yaml
```

其中：

- LOCO 策略路径、关节初始姿态来自 `demo/config/KpiConfig.yaml`
- 仿真 PD 和 MuJoCo base 初始位姿来自 `demo/config/KpiConfig.sim.yaml`

注意：`demo/config/KpiConfig.sim.yaml` 中的 PD 只用于 MuJoCo 仿真，和真机 PD 是独立配置，不能直接当作真机参数使用。

## 本机 NPZ 遥操作回放

发布环境位于 `.venv-pub-motion`，复用系统 NumPy，仅额外安装 `pyzmq`。
在 `sim/demo` 目录中运行以下命令，会先编译并启动 C++ viewer/运动控制，
收到 `tcp://127.0.0.1:28711` 的第一条有效机器人状态后，才启动 50 Hz NPZ 发布端：

```bash
./scripts/run_npz_demo.sh scripts/newsmallstep.npz
```

无 viewer 的实时回放使用 `KPI_NPZ_VIEWER=0`，仍按实时节拍执行：

```bash
KPI_NPZ_VIEWER=0 ./scripts/run_npz_demo.sh scripts/newsmallstep.npz --duration 18
```

默认仍打开 viewer。`jakaminimimic_zmq.clock` 的源时间映射在两种方式下均生效；
接收首个有效包后默认预充约 0.5 秒。完整时间轴、配置和数值说明见
[motion buffer 说明](docs/motion_buffer_timing.md)。

当前默认 `clock.timestamp_mode: source`：新Pico发布器声明 `motion_t_ns`，
在读取XR后、GMR前以发布机单调时钟记录；`publish_t_ns`仍在GMR后、准备发送时生成。
没有源声明的旧NPZ/record报文固定采用publish轴。详细字段与暂停语义见
[发送端时间协议](docs/publisher_protocol.md)。

也可以将文件替换为 `scripts/motion_pico2.npz` 或 `scripts/motion_pico3.npz`。
回放使用 `tcp://127.0.0.1:28701` 和独立生成的配置，保留 NPZ 原始全身关节，
从第 0 帧单次播放。发布端 bind 后默认等待 0.25 秒完成订阅连接握手；
编译和控制器初始化期间不会提前消耗动作帧。
播完后控制器继续运行并保持最后一帧参考，终端会明确提示。
关闭 viewer 或按 Ctrl+C 会清理发布端和本次控制器进程。
原始配置中的远程 Pico 地址仍然保留。
控制器输出同时显示在终端并保存到 `build/npz_replay/sim.log`，
发布日志位于 `build/npz_replay/publisher.log`。
状态消息表示 ZMQ 控制循环已经运行，不等于物理站姿已经完成稳定收敛。

默认等待控制器就绪的超时为 60 秒，可通过 `KPI_NPZ_READY_TIMEOUT` 调整；
`KPI_NPZ_PUBLISHER_STARTUP_SLEEP` 调整发布后的握手等待，
`KPI_NPZ_STATE_CONNECT` 可匹配自定义配置中的状态发布地址。
NPZ 文件名之后的参数继续透传给 C++ demo，例如限制仿真时长：

```bash
./scripts/run_npz_demo.sh scripts/motion_pico2.npz --duration 18
```

重新创建最小发布环境（已有系统 NumPy 和 uv）：

```bash
uv venv --python /usr/bin/python3 --system-site-packages .venv-pub-motion
uv pip install --python .venv-pub-motion/bin/python pyzmq
```

## 本机录制数据回放（Parquet / motion_raw.npz）

复用 `.venv-pub-motion`，额外依赖 `pyarrow` 和 `scipy`。当前环境保留系统
NumPy 1.21.5，因此使用兼容的 SciPy 1.10.1：

```bash
uv pip install --python .venv-pub-motion/bin/python --no-deps pyarrow==25.0.1 scipy==1.10.1
```

默认回放 `20261022-221234/level-0/episode_0000/data.parquet`：

```bash
./scripts/run_record_demo.sh
./scripts/run_record_demo.sh scripts/20261022-221234/level-0/episode_0001
./scripts/run_record_demo.sh scripts/20261022-221234/level-0/episode_0001/motion_raw.npz
```

可传 episode 目录、Parquet/NPZ 文件，或整个 session 目录（选择排序后的第一集）。
启动器先启动 C++ viewer，等其进入 ZMQ 状态并发布有效机器人状态后，再从录制
第 0 帧发布，默认按真实记录时间轴插值到 50 Hz，单次回放；结束后控制端继续保持
最后参考。关闭 viewer 或 Ctrl+C 会清理本次启动的进程。日志在 `build/record_replay/`。
原始数据文件和远程 Pico 配置不作修改，本机回放使用 `127.0.0.1:28701`。

可通过 `--anchor waist|base` 选择发送的 link（默认 `waist`）：

```bash
./scripts/run_record_demo.sh --anchor base
./scripts/run_record_demo.sh scripts/20261022-221234/level-0/episode_0001 --anchor waist
# 也可以直接运行发布端
.venv-pub-motion/bin/python scripts/pub_motion_record.py scripts/20261022-221234/level-0/episode_0000/data.parquet --anchor base
```

`base` 从原始 body[0] 读取位置和 wxyz 姿态；`waist` 从 Parquet 的
`actions[33:40]` 或原始 NPZ 的腰部索引读取。选定 pose 后复制到全部28个 body 槽位，
因此 C++ 读取 `[0]` 或 `[13]` 都得到所选 link 的值，27个关节参考保持原有含义。
也可用 `KPI_RECORD_ANCHOR=base` 设置启动器默认值。

缺少完整 body 列的 Parquet 无法选择 base。导出的占位 NPZ 会记录所选 link，
再次加载时只能选择该 link；切换到另一 link 应使用原始 Parquet 或 `motion_raw.npz`。

可选循环、速度和无界面验证：

```bash
KPI_RECORD_LOOP=1 ./scripts/run_record_demo.sh
KPI_RECORD_SPEED=0.5 ./scripts/run_record_demo.sh scripts/20261022-221234/level-0/episode_0001
KPI_RECORD_VIEWER=0 ./scripts/run_record_demo.sh scripts/20261022-221234/level-0/episode_0000 --realtime --duration 10
```

Parquet 使用 `timestamp_ns`；`motion_raw.npz` 使用 `publish_t_ns`。
旧的录制时间只定义动作时间轴，网络消息里的 `publish_t_ns` 始终重新生成当前时间。
关于字段、时间戳、暂停段和数据质量，见 `RECORD_REPLAY_DATA_AUDIT.md`。

## 编译产物

```text
demo/build/KpiMujocoSimDemo
demo/build/KpiConfig.demo.yaml
demo/build/sim_feedback.csv
```

## 交付边界

如果只给客户 SDK 能力，`sim/sdk` 是库和资源包。

如果需要给客户一个能跑策略的完整仿真样例，则同时给：

```text
sim/sdk
sim/demo
```

其中 `sim/demo` 包含 LOCO 策略运行所需的控制器 demo 源码和策略文件，用于验证 SDK 接入方式。

实际 JAKA 发布端为 `scripts/pico_pub_copy.py`，时间适配保留其首次 X 前静默、
暂停姿态和 A 键采集逻辑；`scripts/pico_retarget_pub.py` 是较早的 G1 参考版本。

仓库初始基线的恢复来源与限制见 [history provenance](docs/history_provenance.md)。
