# Residual160k实机部署配置

2026-11-11。本发布选择 `config/policy_mimic/ckpt_0160000_bones_residual.onnx`，
仅发布模型、配置、说明及离线验证。real的运行时C++与此前已提交版本相同：
旧 `ParKine.cpp`、原失败处理、旧 `RealtimeMotionBuffer` 均保留，
不包含11月10日尚未提交的脚踝优化及配套状态机修改。

模型SHA256：`a35263a6a0ece6788bc6193fed3131723373d906845af4883b528e98de8f6357`。

## 动作协议

输入为float32 `obs[batch,620]`，输出为float32 `actions[batch,27]`。
620维布局及27关节的策略/SDK/buffer映射沿用已有SuperMimic实现。

ONNX自身已经计算：

```
actions = decoder_delta + (q_ref(frame1) - q_default) / 0.5
```

现有C++保持：

```
q_target = q_default + 0.5 * actions
         = q_ref(frame1) + 0.5 * decoder_delta
```

部署侧不再额外加参考角，也不把ONNX输出误当作decoder_delta。
history保存完整raw ONNX输出，随后对物理PR目标按YAML裁切。
frame1是五帧窗口中的第二帧，对应输入切片 `obs[47:74]`。
原buffer查询延迟120ms，窗口为墙钟减去120/100/80/60/40ms，
因此不需要等待一个未来ZMQ包，也不额外增加20ms软件延迟。

## 机器人参数

| 项目 | 配置 |
|---|---|
| 参数来源 | YAML；运行时不从ONNX metadata导入关节/控制参数 |
| 27轴MIT Kp/Kd | 保留原实机换算值；不被训练metadata覆盖 |
| PR Kp（左pitch/roll、右pitch/roll） | `[80,20,80,20]` |
| PR Kd | `[2,1,2,1]` |
| PR pitch目标范围 | `[-.52,.61] rad` |
| PR roll目标范围 | `[-.7,.7] rad` |
| action scale | `.5` |
| Super gyro / joint velocity scale | `.5 / .2` |
| 控制周期 / 未来帧 / history | 20ms / `[0,1,2,3,4]` / 5帧 |
| anchor / reference高度补偿 | `base_link[0]` / 0 |
| 下发方式 | `angle_map=0`，PR目标，原下层PD/力矩映射 |
| 动作接收 | `tcp://10.5.5.120:28701` |

全局旧模式的gyro/velocity scale仍为`.2/.05`，不要为Super修改它们。
训练中的reset加高5cm作用于训练机器人初态，不是部署reference加高。
目标范围不代表扩大机械行程；实机内部限制仍需遵循原控制器配置。

## 构建及使用

应从这个发布提交的干净检出构建，避免混入本机未提交的脚踝优化。
在完整ROS 2 Humble及SDK依赖环境中：
下面的绝对路径为占位符，请替换为该发布提交的干净检出目录。
不要直接在仍含未提交脚踝优化的主工作区real目录编译部署。

```bash
cd /path/to/clean-checkout/real
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build -j4
```

按现有README整理部署包并通过 `startup.sh` 启动。
Super模式仍为使能完成后按 **LB+B** 进入，按 **A** 退出并请求断使能。
发布端继续使用原Pico发送协议，无需因residual改变消息格式或reference时钟。

## 离线核验

使用已有安装了NumPy、PyYAML、ONNX、ONNX Runtime的Python：

```bash
python tests/check_supermimic_yaml.py \
  --model config/policy_mimic/ckpt_0160000_bones_residual.onnx \
  --config config/KpiConfig.yaml --require-reference-residual
```

此检查允许读取metadata做离线核对，但运行时仍完全使用YAML参数。
检查27关节顺序、默认角、尺度、专用PR增益，并运行128组真实ORT输入，
从图中同时取得decoder_delta和完整actions，验证skip只计算一次。
实机换算MIT参数按原授权配置保留，其内部换算公式不由本检查重新推导。

CMake新增residual模型的YAML合同及策略数学测试，同时保留旧模型测试。
未构造RobotIntf的离线测试不下发硬件命令；协议/数值一致不等于实机运动稳定性已验证。
此版本按要求保留旧踝求解器，因此此前发现的旧分支求解隐患并未被本发布修复。

本轮离线结果：

- 模型内嵌skip实际图验证128组输入，raw action恒等式最大误差0，
  等价PD target公式最大误差2.384e−7rad。
- 实际旧real FSM、真实ORT推理，对Pico2/Pico3/Dance各500tick重放，
  obs/action及按real YAML独立裁切目标的最大误差均为0。
- 候选版本5项CTest通过；实际KpiConfigInit保留YAML的增益、默认角和限幅。
- 数值重放使用解码后的PR状态和参考窗口，不覆盖编码器解算、live网络时钟、
  下层PD或实机稳定性。硬件I/O被禁用适配器阻断，推理没有mock。
- 本机完整硬件程序的最终链接仍需原SDK依赖
  `libstd_srvs__rosidl_typesupport_cpp.so`；未启动硬件程序。
