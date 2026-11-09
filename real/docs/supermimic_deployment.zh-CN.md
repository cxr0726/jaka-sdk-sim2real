# Khan SuperMimic 实机迁移

2026-11-09 当前配置：默认模型切换为 BONES 100k，专用 PR PD 改为 pitch `80/2`、roll `20/1`，roll 虚拟目标边界改为 `[-0.7,0.7] rad`。顶层27轴换算后的 MIT 参数、策略关节顺序、观测尺度和旧 buffer/并联解算路径保持。以下历史迁移、回退和闭环比较记录使用当时的模型及参数，不能当作新配置的实机运动验证。

2026-11-03。迁移前基线为上层仓库 `eefd5ed`。该仓库只跟踪 `sim/`、`real/` 和根 `.gitignore`；`sim/.git` 原历史保留，不将 sim 记录为 gitlink。虚拟环境、Git 元数据、构建/缓存/日志及日期命名的个人录制目录忽略。

本次按用户要求，实机 SuperMimic 恢复使用原 `RealtimeMotionBuffer` 和原 `AB2PitchRoll` 并联正解/失败置零行为。此前迁移的源时间映射、generation、预充、参考额外加高和修复版并联求解器不再接入实机 SuperMimic；sim 不作此回退。关节映射、策略观测/输出结构和 YAML 参数来源保留。回退后的验证结果单独记录；文末旧结果归为历史证据。

## 模型及 YAML 参数来源

新增 `FSMMimicJakaMiniSuperZmq.cpp`、`FSM_JAKAMINIMIMIC_SUPERZMQ=13`，布局沿用 sim。当前默认模型为 `config/policy_mimic/ckpt_0100000_bones.onnx`，SHA256 `6f6dfb923bf6597572fed243f3bfe364a0635b50409896429a9ed19d917307e8`，输入 float32 `obs[1,620]`，输出 `actions[1,27]`。

同时附带 `config/policy_mimic/ckpt_0120000_bones_delay.onnx` 作为可选模型。两者的 actor 部署协议、关节顺序、默认角、尺度和标称 PR PD 相同；选用120k时只需把 `jakaminimimic_superzmq.policy_path` 改成 `policy_mimic/ckpt_0120000_bones_delay.onnx`。`delay` 是训练版本标记，不需要在部署端额外增加软件延迟，也不代表两个策略的输出或运动效果相同。

120k SHA256 为 `59827831e16ae6c5d1139a161aa350261b39326814fbb6767f42fe7dadf19746`；sim、real 中的同名模型逐字节相同。两模型均为 float32 `obs[batch,620] -> actions[batch,27]`，完整部署 schema 一致：JAKA 关节顺序、base_link、20ms 控制周期、5帧未来/逐项历史、action scale `.5`、gyro `.5`、joint velocity `.2`。120k 的训练合同改进了执行器延迟队列的 reset 预置，不增加 actor 输入或 hidden state。两模型的 `reference_height_offset` 均为0；训练合同中的 `reset_root_z_offset=.05` 是实际机器人 reset 加高，不能转换成部署参考加高。

### 2026-11-09 当前配置验证

- real 完整构建及6项 CTest、sim 完整构建及10项 CTest 全部通过。
- 两模型各自通过实际 real 加载器和 `KpiConfigInit`/warmup 检查，确认新 PR PD、roll ±.7 及原27轴 MIT 参数按 YAML 生效；120k 使用仅更改模型路径的诊断配置。
- 每模型对 Pico2、Pico3、dance 各回放500个相同输入 tick，共1500 tick；当前 sim/real FSM 的620维观测、action/history、原始目标和按新 real 边界裁切后的目标，独立检查最大误差为0。该比较是每个模型在两套部署实现之间的对照，不是要求两个不同模型输出相同。
- 两模型各完成一次18.02秒 Pico2 无 viewer 实时 ZMQ 仿真，各898个策略 tick，`err=0`，记录值全部有限且未倾倒。两次使用默认 XML、发布端高度补偿0、sim目标限幅 `none`；新 real ±.7 边界在前述配置初始化和离线目标计算中验证，此仿真不代表新 real 限幅下的实机闭环效果。

上述 real 测试均未构造硬件接口或下发实机指令。协议与计算兼容性通过，不代表已经验证实机运动稳定性。

### 参数来源

实机 `SuperMimicDeployment` **不读取 ONNX metadata**，仅检查 tensor 的数量、名称、类型、shape。控制参数及关节顺序都来自 YAML，导出 JSON 不是运行时依赖。

| YAML 字段 | 顺序和用途 |
| --- | --- |
| 顶层 `joint_names` | 原 SDK/控制器部署顺序 |
| 顶层 `default_angles` | SDK序 q0，按名字重排；用于 q−q0 和输出目标 |
| 顶层 `init_target` | 原实机初始化姿态，不被模型覆盖 |
| 顶层 `kp/kd` | 用户已转换的实机 MIT 参数，保留全100 / 100或200 |
| 顶层 `kp_pitch_roll/kd_pitch_roll` | BONES 专用 PR PD：`[80,20,80,20]` / `[2,1,2,1]`；左pitch/roll、右pitch/roll |
| 新小节 `joint_names` | 模型输入输出的27关节策略顺序，显式配置 |
| 新小节 `action_scale` | 当前 .5，支持标量或27维策略序数组；缺省继承顶层值 |
| 新小节 `ang_vel_scale/dof_vel_scale` | 当前 .5/.2；旧模式顶层 .2/.05 保持 |

例如右 hip pitch 为 policy6→SDK12，左 shoulder roll 为 policy14→SDK7。参考从 buffer 的 Isaac 顺序独立按名字 gather，输出按名字 scatter，历史动作保持策略序。更换模型时可用离线 checker 检查手工配置的顺序。

27关节的 policy/SDK/buffer/wire 索引、默认角及实际控制通道核对表见迁移时保存的 [supermimic_joint_contract.json](supermimic_joint_contract.json)；其中旧 PR 增益属于历史快照，当前增益以上述 YAML 为准。四踝区分全局 YAML 增益与真正启用的 MIT/PR 两路参数。

实机 `FSMInit.cpp` 未修改：`angle_map=0` 时四踝普通 MIT gains 原本就设为 `(0,5)`，然后启用专用 PR PD；没有复制 sim 的 `(0,0)`，也不从 ONNX 自动覆盖实机参数。当前 PR PD 是在 YAML 中显式更新的 BONES 参数。

原始 action 不做 ±5 裁剪，不加 tanh/平滑。目标为 `q0 + action_scale * raw_action`，按实机既有物理关节位置限幅约束，发送 PR 目标；历史保存 raw action。**实机 SuperMimic 只支持 `angle_map=0` 的专用 PR PD 路径**。非零值进入时故障断使能，运行中若值被改变也不发目标。旧模式的 angle_map 行为不变。

顶层四维 `pitch_roll_down_limit/up_limit` 在 `angle_map=0` 时控制虚拟 PR PD 目标边界，当前为 `[-.52,-.70,-.52,-.70]` / `[.61,.70,.61,.70]`；pitch 保留原边界，roll 从原先 `±.35` 放宽到 `±.70`。这不会更改机械行程或控制器内部限制。其余关节仍使用 `down_limit/up_limit`。这些专用 PR 增益及边界是全局配置，也影响使用同一执行路径的普通 ZMQ 等模式。

## 观测和实时参考

620维为 goal155 + 未来参考相对姿态30 + 五帧传感器/动作历史435；窗口 `[0,1,2,3,4]`、控制20ms，history逐项从旧到新。重置时 previous action 为0，传感器首样本重复五次。

参考和测量锚点为 base_link，发送数组索引固定0。IMU 沿用实机 `imu_process` 安装轴校准，不追加腰 yaw/dyaw。缺失/非有限 IMU、异常四元数或无效关节反馈会故障，不伪造 identity/零测量继续推理。当前 `get_imu()` 没有接收年龄接口，不能宣称新增了 IMU 新鲜度 watchdog。

当前使用与旧实机 ZMQ 模式相同的 `RealtimeMotionBuffer`：JSON 解析后取接收端 `system_clock::now()` 作为插入时间，`get_obs()` 也按本机墙钟查询。`dt_s=.02`、`future_steps=[0,1,2,3,4]`、`tolerance_s=.04` 对应查询延迟 `4×20+40=120ms`，五帧查询时间为当前墙钟减去 `[120,100,80,60,40]ms`。窗口外端点截取，位置/关节线性插值、姿态 SLERP；没有源时间映射或500ms预充。

默认接收 `tcp://10.5.5.120:28701`，根参考使用发送数组索引0，不额外加高5cm；已移除实机 Super YAML 的 `clock` 和 `reference_height_offset`。第一控制步对当前参考（无包时为默认参考）对齐 yaw，并初始化 history/previous action；首次有效包使 `ready()` 从 false 变 true 时仅再次对齐 yaw，继续推进已有传感器/动作历史，不将其清空，沿用旧 ZMQ 行为。空缓冲返回配置的站立参考；收到包后，断流继续保持最后参考，没有 stale 超时或新流 generation 重置。此处完全恢复旧 buffer 行为，不代表新增了断流保护。

### 仿真端单独启动控制

以下只启动 SuperMimic 控制并等待 ZMQ 动作，不启动发布端：

```bash
cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
./scripts/run_demo.sh --viewer --duration 0 --mode jakaminimimic_superzmq
```

`--viewer` 自动启用实时节拍；无 viewer 时将其替换为 `--realtime`。默认 XML 为 `sim/sdk/assets/khan_mini/Khan_mini_contact.xml`（当前完整脚部凸包），由 `KPI_MUJOCO_XML` 环境变量或显式 `--xml` 覆盖。当前 sim YAML 仍选择 `config/policy_mimic/ckpt_0080000_khan_mixed_dr_v6.onnx`，连接 `tcp://127.0.0.1:28701`；本次 real 的模型切换不会修改这两个 sim 默认值。SDK 根目录也可由 `KPI_MUJOCO_SIM_SDK_ROOT` 改写。

指定 BONES120k 并启动本机 NPZ 播放可使用（先启动控制器，再发送动作）：

```bash
./scripts/run_super_npz_demo.sh scripts/motion_pico2.npz \
  --policy config/policy_mimic/ckpt_0120000_bones_delay.onnx \
  --viewer --duration 0 --target-limits real --height-offset 0
```

这里 `--target-limits real` 启用的是 **sim 自己的目标限幅**，并不读取 `real/config/KpiConfig.yaml`。当前 sim 的 roll 边界仍为 `±0.35`，本次 real 已改为 `±0.70`；因此这条命令与新 real 配置的目标限幅并不完全相同。`--policy` 仅修改该次生成的回放配置，`--height-offset 0` 明确禁止发布端额外加高。

### PICO 实时发送端

使用 `sim/demo/scripts/pico_pub_copy.py`，在原先已经配置好的 JAKA/Pico/GMR 发送端环境运行：

```bash
cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
python scripts/pico_pub_copy.py --robot jaka --bind 'tcp://*:28701'
```

脚本默认 robot=g1，必须指定 `--robot jaka`，其 GMR 目标映射为 jaka_mini。默认发布30Hz，C++策略仍50Hz并由 buffer 插值；需要50Hz时可加 `--publish-hz 50`，实际频率受XR/GMR耗时影响。real 当前接收配置指向发送设备 `10.5.5.120`；sim 若跨机器接入，也需将自己的 `zmq_connect` 从本机地址改成发送设备地址。`bind *` 只表示监听发送设备所有网卡，不会改变它的IP。

脚本启动后首次 PICO X 前静默；控制器已就位并进入策略后按X开始遥操作，后续X暂停/恢复，暂停仍发布保持姿态和新 motion_t_ns。PICO A 在本脚本中只切换采集标志，不直接调用实机断使能；实机结束按 real 框架接收的遥控 A。不要加 `--skip-retarget`，该调试分支产生 NaN 关节，不能用于策略部署。

发送端仍在XR读取边界、GMR之前记录 motion_t_ns，在payload完成后记录 publish_t_ns；当前实机原 buffer 不读取这两个源时间戳，而是按本地接收墙钟插入。此协议仍可供未回退的 sim source 模式使用，无需回退发送脚本。

本仓库不包含完整 JAKA 发送端环境：附带 thirdparty/sim2real 的上游注册表只有G1/H2，而此脚本从实际 Python 环境中的 sim2real 获取 robot_cfg；必须使用原来含JAKA配置和GMR资产的环境。其 joint_names 应为27维JAKA发送序，body_names[0]为base_link。不能将一份未安装JAKA扩展的上游环境当作可直接运行的发送端。

### Buffer 选择与旧版区别

当前实机三个模式均使用原 `RealtimeMotionBuffer`。实机目录的 `SuperRealtimeMotionBuffer`、`SourceTimeline` 实现随本次回退移除；当前 sim 普通 ZMQ 和 Super 也使用本地时间轴的 `LegacyRealtimeMotionBuffer`，仅 sim fixwrist 保留源时间映射实现。

| real 模式 | 从 IDLE 进入 | 使用的 buffer |
| --- | --- | --- |
| SuperMimic / superzmq | LB+B（先释放再按） | 原 `RealtimeMotionBuffer` |
| 普通 zmq | LB+START | 原 `RealtimeMotionBuffer` |
| zmq_fixwrist | LB+BACK | 原 `RealtimeMotionBuffer` |

本次回退前，Super 使用源时间映射、原子 MotionWindow、预充、stale/underrun 和 generation；本次直接切回旧 buffer，并非将新版 `clock.timestamp_mode` 改成 arrival。原 buffer 没有这些状态，也不读取源时间协议或额外高度补偿。

没有运行时 buffer 类切换开关。当前实机 Super 的 YAML 不再读取 `clock.*`/`reference_height_offset`；重新加入这些键不会启用新版时间轴。模型/关节参数等有效配置更改仍需重启程序。

## 状态机操作

保留原 INIT→IDLE 启动流程，不自动使能或进入策略。按原流程使能所有关节、就位后：

- IDLE 先松开组合键，再按 **LB+B** 进入 SuperMimic；要求 JOY 在线、关节全部使能、无故障，进入时再检查反馈和 IMU。
- SuperMimic 先松开、再按 **LB+B** 返回 IDLE，释放本模式缓冲和状态发布。
- **A** 请求断使能并进入 FAULT；新模式下 JOY 所有权丢失或手柄 watchdog 离线同样故障。

正常遥操作结束直接在 Super 中按 A，不需要先回 IDLE。同一 FSMUpdate 会切到 FAULT，该周期不再推理或发送轨迹。A 的优先级及进入/退出防误触逻辑继续保留；此前51个离线用例的结果属于回退前验证，当前结果见回退后的独立记录。

释放后再按可避免旧 MIMIC 的 LB+B 返回手势误触发新模式。原 LB+Y、LB+X、LB+START、LB+BACK 保持。进入后即执行平衡/跟踪，RB 不作为额外启控开关。实机主线程原有20ms调度未修改，周期从 YAML 校验，不从 metadata 设置。

## 并联解算与错误处理

当前 Super 恢复调用原 `ParKine.cpp` 的 C ABI `AB2PitchRoll`，与旧 ZMQ/fixwrist 一致。左脚传入原AB反馈，右脚传入取负后的AB位置/速度；任一脚求解返回负值时，该脚 pitch、roll 及对应速度置零，继续后续流程。此前独立有界求解器的缓存、分支交叉检查及“求解失败即停机”不再应用于 Super；失败置零是用户要求恢复的旧行为，保留了失真观测的风险。

没有修改原 `ParKine.cpp` / C ABI，也没有改控制器进程/硬件库内部 PR PD 算法。有效反馈/IMU检查、观测/推理输出有限值检查、实机限幅、轨迹 API 失败处理及 A/watchdog 停机仍保留。

当前 angle_map=0 时，旧 ZMQ/fixwrist 与 Super 都下发物理 PR 目标，不在本进程先转换成AB；由原控制器专用PR PD处理。右脚PR目标不额外取负。最终都是 `set_joint_trajectory(mJointNames,mPt)`，20ms周期、SDK关节顺序和实机PD配置相同。策略输出语义仍不同：旧模式 raw action 先裁到±5，Super保留raw值/历史，再对物理目标限幅。Super 继续检查轨迹API拒绝并请求停机。

## 构建及回退后验证

依赖原有 ROS 2 Humble（含 RobotIntf 所需 std_srvs）、Eigen、yaml-cpp 和随附库。移除未使用的 rclcpp_components 构建要求，补充 sensor_msgs/std_srvs 依赖声明及 ZMQ soname 符号链接。

```bash
cd /home/cxr0726/workspace/JAKA_comparison/real
source /opt/ros/humble/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

上传和启动沿用原 README 的部署目录、startup.sh 及 ROS 配置。本次没有运行 startup.sh/KpiDeployReal，没有向控制器发消息。

回退后 `KpiDeployReal` 完整编译链接通过，但没有运行。6项 CTest 全通过，实际 FSM 覆盖53个用例；验证原 buffer 的接收/查询墙钟、源时间字段忽略、首包立即 ready、body[0]/无额外加高、关节重排、插值和末帧保持，以及原 AB2 解码符号、负返回单腿置零继续、首次 ready 仅重对齐 yaw 并保留 history/action。模型映射、YAML增益/限位、实际策略观测/输出数学和A键路径也通过离线检查。原 `ParKine.cpp`、`RealtimeMotionBuffer.cpp/.hpp` 与迁移前 `eefd5ed` 逐字节一致，sim 未改。详见 `supermimic_validation.json` 的 `current_validation`。

另有32步合成 legacy 就绪事件回放，经实际 FSM 数学路径和训练独立公式核对通过；首包只重对齐而不清历史，观测最大误差2.78319e-7，CPU ORT action和YAML目标误差为0。此测试不模拟网络或原始AB传感器。

这些测试不验证实机闭环效果。回退前源时间/修复求解器单元测试的通过结果也不能视为当前 Super 实机路径的验证。

## 回退前的历史验证（不代表当前配置已验证）

以下结果保留自回退前版本；完整数据移至 `supermimic_validation.json` 的 `pre_rollback`。尤其源时间、generation/stale 和修复求解器的通过记录不适用于当前原 buffer/原并联正解路径。

本机 `build-supermimic/KpiDeployReal` 完整链接通过。宿主缺少 std_srvs，本次从已配置且签名验证的官方 ROS 索引下载对应包，**仅解压 /tmp** 供链接/离线测试，没有安装系统软件。临时 rpath-link/LD_LIBRARY_PATH 只用于本机验证，不写入源码或部署路径。

7项 CTest 全通过：YAML关节映射/metadata无关性、2601姿态并联安全、源时间映射、buffer协议、实际FSM数学、实际KpiConfigInit初始化/释放与PD保持、实际状态机51个故障/切换用例。删除 ONNX 全部 metadata 后 adapter 仍按 YAML 正确工作。

相同起身片段反馈/参考离线回放实际 FSM 1600步，未构造 RobotIntf、初始化ROS或创建ZMQ socket：

| 比较 | 最大绝对误差 |
| --- | ---: |
| C++ 观测 vs 独立训练公式 | 6.03497e-7 |
| C++ action vs CPU ORT | 0 |
| YAML默认角/缩放/重排/限位后目标 | 0 |
| C++ action vs 当前 PT/Torch | 4.29153e-6 |

两次 history reset 正确；550个目标分量触发实机限幅，raw action/history保持。该结果验证同输入下的计算，尚未测试实机闭环运动效果。

可用已有 Python+NumPy+PyYAML+ORT 环境运行配置核对：

```bash
python tests/check_supermimic_yaml.py \
  --model config/policy_mimic/ckpt_0100000_bones.onnx \
  --config config/KpiConfig.yaml
```

`tests/check_supermimic_runtime.py` 可用训练环境和 PT 独立核对诊断 tape；只有离线脚本读取 metadata 检查训练契约，运行时不读取。原始记录在忽略的 build-supermimic，简要结果为 `supermimic_validation.json`。

### 历史 sim 与 real 实际函数直接对照

两边离线驱动调用各自实际 FSM 的观测、历史、推理和动作函数，使用同一个新 ONNX、同一反馈/参考；旧录制只作为输入，并重新使用新策略推理，没有混用旧策略 action。安全修订后重建 real 驱动、重新核对：

| 输入 | 控制步数 | 620维各观测块 / raw action 最大差 | real 与按实机限幅后 sim 目标最大差 |
| --- | ---: | ---: | ---: |
| fallAndGetUp2_subject2 | 8316 | 0 | 0 |
| walk2_subject1 | 12197 | 0 | 0 |
| dance1_subject2 | 6797 | 0 | 0 |
| newsmallstep | 3127 | 0 | 0 |
| motion_pico2 | 741 | 0 | 0 |
| 合计 | 31178 | 0 | 0 |

实机限幅仍是有意差别。getup 共5524个目标分量被限幅，最大裁剪量3.582rad；观测/action计算一致不能推出两端物理闭环效果必然相同。没有为追求数值相等而取消实机限位。详细比较脚本 `tests/check_sim_real_supermimic.py` 独立按名称匹配关节并检查限幅前后公式。

本次还修复两项边界问题：Super 返回 IDLE 后本地 enable 标志被重置导致 A 可能被忽略，现 A 不依赖该标志、优先于 BACK 和所有模式组合键；备用 angle_map=1 的 PR→AB 后电机空间限幅不完整，现 Super 明确拒绝该路径。新增用例在旧482cf1e上能复现第一项失败，在修复版本上全部通过。失败进入策略、已有故障时，同周期不会继续推理/发轨迹。
