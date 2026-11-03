# Khan SuperMimic 实机迁移

2026-11-03。迁移前基线为上层仓库 `eefd5ed`。该仓库只跟踪 `sim/`、`real/` 和根 `.gitignore`；`sim/.git` 原历史保留，不将 sim 记录为 gitlink。虚拟环境、Git 元数据、构建/缓存/日志及日期命名的个人录制目录忽略。

## 模型及 YAML 参数来源

新增 `FSMMimicJakaMiniSuperZmq.cpp`、`FSM_JAKAMINIMIMIC_SUPERZMQ=13`，布局沿用 sim。模型为 `config/policy_mimic/ckpt_0100000_new_supermimic.onnx`，SHA256 `67877118414560c830491e43fdeb405011a222aecf75310be368c8d1cbef4c6b`，输入 float32 `obs[1,620]`，输出 `actions[1,27]`。

实机 `SuperMimicDeployment` **不读取 ONNX metadata**，仅检查 tensor 的数量、名称、类型、shape。控制参数及关节顺序都来自 YAML，导出 JSON 不是运行时依赖。

| YAML 字段 | 顺序和用途 |
| --- | --- |
| 顶层 `joint_names` | 原 SDK/控制器部署顺序 |
| 顶层 `default_angles` | SDK序 q0，按名字重排；用于 q−q0 和输出目标 |
| 顶层 `init_target` | 原实机初始化姿态，不被模型覆盖 |
| 顶层 `kp/kd` | 用户已转换的实机 MIT 参数，保留全100 / 100或200 |
| 顶层 `kp_pitch_roll/kd_pitch_roll` | 保留 `[100,50,100,50]` / `[2,.5,2,.5]` |
| 新小节 `joint_names` | 模型输入输出的27关节策略顺序，显式配置 |
| 新小节 `action_scale` | 当前 .5，支持标量或27维策略序数组；缺省继承顶层值 |
| 新小节 `ang_vel_scale/dof_vel_scale` | 当前 .5/.2；旧模式顶层 .2/.05 保持 |

例如右 hip pitch 为 policy6→SDK12，左 shoulder roll 为 policy14→SDK7。参考从 buffer 的 Isaac 顺序独立按名字 gather，输出按名字 scatter，历史动作保持策略序。更换模型时可用离线 checker 检查手工配置的顺序。

27关节的 policy/SDK/buffer/wire 索引、默认角及实际控制通道核对表见 [supermimic_joint_contract.json](supermimic_joint_contract.json)；四踝区分全局 YAML 增益与真正启用的 MIT/PR 两路参数。

实机 `FSMInit.cpp` 未修改：`angle_map=0` 时四踝普通 MIT gains 原本就设为 `(0,5)`，然后启用专用 PR PD；没有复制 sim 的 `(0,0)`，没有将训练 PD 覆盖到实机。

原始 action 不做 ±5 裁剪，不加 tanh/平滑。目标为 `q0 + action_scale * raw_action`，按实机既有物理关节位置限幅约束，发送 PR 目标；历史保存 raw action。**实机 SuperMimic 只支持 `angle_map=0` 的专用 PR PD 路径**。非零值进入时故障断使能，运行中若值被改变也不发目标。旧模式的 angle_map 行为不变。

新增顶层四维 `pitch_roll_down_limit/up_limit`，以 YAML 明确保存此前代码写死的物理 PR 限位 `[-.52,-.35,-.52,-.35]` / `[.61,.35,.61,.35]`；当前 `angle_map=0` 实际限幅不变，其余关节使用 `down_limit/up_limit`。本次离线验证覆盖当前 `angle_map=0` 配置。

## 观测和实时参考

620维为 goal155 + 未来参考相对姿态30 + 五帧传感器/动作历史435；窗口 `[0,1,2,3,4]`、控制20ms，history逐项从旧到新。重置时 previous action 为0，传感器首样本重复五次。

参考和测量锚点为 base_link，发送数组索引固定0。IMU 沿用实机 `imu_process` 安装轴校准，不追加腰 yaw/dyaw。缺失/非有限 IMU、异常四元数或无效关节反馈会故障，不伪造 identity/零测量继续推理。当前 `get_imu()` 没有接收年龄接口，不能宣称新增了 IMU 新鲜度 watchdog。

新增 `SuperRealtimeMotionBuffer`、`SourceTimeline`，复用 sim 的源时间映射、原子窗口和 generation 状态。原 `RealtimeMotionBuffer` 及旧实机模式保持原实现。默认 source clock，首有效包后预充500ms，查询延迟120ms；支持声明的采样时间字段及旧流 publish_t_ns。进入模式首步、新流预充就绪时重建 yaw/history/previous action。

默认接收 `tcp://10.5.5.120:28701`，`reference_height_offset=.05` 保留现场 Pico 校准；已校准训练 NPZ 应使用0。缓冲缺流/过期时使用明确的默认/保持参考继续平衡；这不等于伪造机器人测量。

### Buffer 选择与旧版区别

这里的“旧版”指 real 的 `RealtimeMotionBuffer`；当前 sim 的 `RealtimeMotionBuffer` 核心算法与 real 的 `SuperRealtimeMotionBuffer` 相同，real 另起类名是为了保留原两种部署模式的行为。

| real 模式 | 从 IDLE 进入 | 使用的 buffer |
| --- | --- | --- |
| SuperMimic / superzmq | LB+B（先释放再按） | `SuperRealtimeMotionBuffer` |
| 普通 zmq | LB+START | 原 `RealtimeMotionBuffer` |
| zmq_fixwrist | LB+BACK | 原 `RealtimeMotionBuffer` |

旧 real buffer 在 JSON 解析后取本机 `system_clock::now()` 作为插入时间，查询也按本机墙钟；`get_obs()` 只返回5帧。新版在收包时立即记录 steady_clock，保存/映射源时间轴，返回一次原子的 MotionWindow（帧、generation、时间、覆盖和状态），有预充、stale、underrun、会话/序号/异常时间处理。位置/关节线性插值和姿态 SLERP 的基本计算相同。新版还从 YAML 明确读取参考高度校准；旧 real buffer 没有额外加5cm，新 Super 配置当前为+.05，需按发送端是否已校准选择，不能重复补偿。

选择 FSM 会自动选择类，没有单独的“buffer 类切换”开关。新模式 `clock.timestamp_mode` 可选 source/publish/arrival；source 使用声明的时间轴（旧流锁定 publish_t_ns），publish 强制 publish_t_ns 再做映射，arrival 使用接收单调时间。这只切换新版的时间策略；arrival 仍保留预充/会话/过期等机制，不等同完全切回旧 buffer。配置更改需重启程序。

## 状态机操作

保留原 INIT→IDLE 启动流程，不自动使能或进入策略。按原流程使能所有关节、就位后：

- IDLE 先松开组合键，再按 **LB+B** 进入 SuperMimic；要求 JOY 在线、关节全部使能、无故障，进入时再检查反馈和 IMU。
- SuperMimic 先松开、再按 **LB+B** 返回 IDLE，释放本模式缓冲和状态发布。
- **A** 请求断使能并进入 FAULT；新模式下 JOY 所有权丢失或手柄 watchdog 离线同样故障。

正常遥操作结束直接在 Super 中按 A，不需要先回 IDLE。同一 FSMUpdate 会切到 FAULT，该周期不再推理或发送轨迹。51个离线状态机用例覆盖 A 单按及 A 与各模式键同时按下。

释放后再按可避免旧 MIMIC 的 LB+B 返回手势误触发新模式。原 LB+Y、LB+X、LB+START、LB+BACK 保持。进入后即执行平衡/跟踪，RB 不作为额外启控开关。实机主线程原有20ms调度未修改，周期从 YAML 校验，不从 metadata 设置。

## 并联解算与错误处理

新模式使用独立的 sim 修复版有界求解器和 `SuperMimicSafety`：左右独立缓存，缓存解与确定性解交叉检查，两腿成功后才提交，保留右脚符号/速度约定。失败设置 ERR_JOINT、请求关闭 PD/断使能，不用零解或旧解继续发轨迹。还检查观测、推理异常/输出和轨迹 API 返回值。

没有替换旧 `ParKine.cpp` / C ABI，没有改变其他模式的解算，没有改控制器进程/硬件库内部 PR PD 算法。

## 构建与验证

依赖原有 ROS 2 Humble（含 RobotIntf 所需 std_srvs）、Eigen、yaml-cpp 和随附库。移除未使用的 rclcpp_components 构建要求，补充 sensor_msgs/std_srvs 依赖声明及 ZMQ soname 符号链接。

```bash
cd /home/cxr0726/workspace/JAKA_comparison/real
source /opt/ros/humble/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

上传和启动沿用原 README 的部署目录、startup.sh 及 ROS 配置。本次没有运行 startup.sh/KpiDeployReal，没有向控制器发消息。

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
  --model config/policy_mimic/ckpt_0100000_new_supermimic.onnx \
  --config config/KpiConfig.yaml --baseline-ref eefd5ed
```

`tests/check_supermimic_runtime.py` 可用训练环境和 PT 独立核对诊断 tape；只有离线脚本读取 metadata 检查训练契约，运行时不读取。原始记录在忽略的 build-supermimic，简要结果为 `supermimic_validation.json`。

## 加深核查：sim 与 real 实际函数直接对照

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
