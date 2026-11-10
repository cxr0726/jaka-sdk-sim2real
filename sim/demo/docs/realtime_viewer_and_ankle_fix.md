# 实时渲染与并联踝正解修复

2026-11-10。本次修复针对实测的两条问题链路：同步 VSync 渲染使每20ms物理积分占用约33.3ms墙钟；旧AB→PR正解返回有限但错误的周期/装配分支，污染SDK PD及策略反馈。与2026-11-04回退后的行为不同。

## 渲染与控制

GLFW/OpenGL窗口、事件处理、渲染仍在主线程；FSM、策略、SDK物理与实时等待在控制线程。无viewer继续在原主线程执行控制。渲染持有独立 `mjModel` 和 `mjData`，不读取正在积分的backend；控制只在完整tick后尝试发布快照，mailbox忙则跳过显示帧，不等待渲染。

默认VSync关闭、显示60fps。`--viewer-vsync`可以只对显示开启VSync，`--viewer-fps 5`也只降低显示刷新率，不把50Hz控制降到5Hz。窗口关闭、异常或Ctrl-C时先停止并join控制线程，再关闭PD/使能和销毁资源。未改SDK动态库。

关闭VSync本身不能保证渲染始终小于20ms；线程解耦移除了渲染阻塞控制的依赖。CPU调度、策略或物理计算仍可能超周期。`--timing-log file.csv`记录实际start-to-start间隔、控制工作耗时及相对实时deadline的落后量；`work_before_log_ms/lag_before_log_ms`不包括该行CSV/告警I/O，`period_ms`包含上一轮I/O和调度影响。控制落后超过一周期时有节流告警。

## 正解及错误处理

`AB2PitchRoll`保留C ABI和左右符号约定，使用此前已验证的 `ParallelAnkleSolver` stateless有界Newton核心。与本轮仅为因果诊断的legacy-fallback gate不同，生产每次正解都检查残差、装配域和Jacobian条件数。pitch±1.4/roll±1.2的求解域用于排除数学分支，不是允许实机target达到这些角度。

原 `PitchRoll2AB` inverse未随本次修改。motion buffer、reference高度、关节顺序、ONNX action/residual协议、PD、target limit及碰撞XML均未改变。

仿真 `CheckedMujocoPhysics` 在每1ms SDK PD前用同一stateless正解预检当前实际AB状态，失败关闭PD/使能、清ctrl、停止推进。主程序继续导出forward符号，SDK动态库内部和策略侧使用同一修复结果。策略侧双腿都成功后才提交反馈，失败锁存fault，不能将零角或上一解伪装成当前有效反馈。

real外部策略反馈使用相同有界正解和失败停控，普通ZMQ、fixwrist、Super等共享漏洞一并修复。真实下层 `legged_waist_controller` 源码不在本仓库，不声称本次修改覆盖其内部1kHz PD/力矩映射。real参数继续来自YAML。

## 播放命令

```bash
cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
./scripts/run_super_npz_demo.sh scripts/motion_pico3.npz \
  --policy config/policy_mimic/ckpt_0100000_bones.onnx \
  --viewer --duration 0 --target-limits none --height-offset 0
```

Residual160k将policy替换为 `config/policy_mimic/ckpt_0160000_bones_residual.onnx`；动作可替换为 `scripts/dance1_subject2.npz`。无需为residual额外叠加reference。

诊断显示与控制解耦（有意降低显示到5fps并开启VSync）：

```bash
./scripts/run_super_npz_demo.sh scripts/motion_pico3.npz \
  --policy config/policy_mimic/ckpt_0160000_bones_residual.onnx \
  --viewer --viewer-fps 5 --viewer-vsync --duration 25 \
  --target-limits none --height-offset 0 --timing-log /tmp/khan_control_timing.csv
```

`--duration`仍是模拟时间；`--no-viewer`用于无界面比较。启动脚本会重新构建，先启动控制，再从NPZ第0帧发布。

## 碰撞配置边界

当前SDK和160k训练均为整脚凸包，左右脚互碰均开启。训练两足mask1/1、priority1；SDK左2/23、右4/15、priority0。SDK的mask屏蔽同侧脚与distal小腿凸包的嵌套接触；只拷贝训练脚mask到SDK会重新开启这两对，而训练本身没有相同distal几何。priority控制材质参数混合，不是碰撞开关。本次保留XML，未将训练mask直接套到SDK几何。

原始录制、旧版对照、数值消融及修复后回归结果位于工作区 `diagnostics/residual_cpp_mjlab_20261110/`。实机未运行。

## 本轮验证

sim完整构建和11项CTest通过，包括viewer快照隔离/并发一致性、两组Dance错误分支、Jacobian及左右符号、SDK内部实际PD使用修复ABI、故障不推进物理/清ctrl、反馈不部分提交。real另通过5项测试，包含进入模式失败后同tick不再执行StateRun，以及INIT绑定失败不再进入IDLE。

real生产源文件均编译成功，但当前环境缺少RobotIntf依赖的 `libstd_srvs__rosidl_typesupport_cpp.so`，完整程序及Policy/Config两项测试无法最终链接。没有将其宣称为通过，未构造或启动实机控制。

Residual160k、Pico3、默认SDK接触模型、真实50Hz ZMQ、无target clipping、参考高度0：

| 条件 | 控制间隔中位数 | p99 | 墙钟/物理推进比 |
|---|---:|---:|---:|
| 无viewer | 20.0001ms | 20.114ms | 1.0000003 |
| viewer60fps、VSync关 | 20.0000ms | 20.073ms | 1.0000001 |
| viewer5fps、VSync开 | 20.0000ms | 20.071ms | .9999999 |

三组均正常完成Pico3，踝反馈对原生物理PR的最大误差约.0072rad。无viewer有一次26.6ms调度间隔，故不保证操作系统不存在单次miss。

同配置完整Dance运行134.46秒模拟时间（参考131.46秒），err=0，最终root高度.63465m/up=.99282，无倒地；踝解码对原生PR全片最大误差.01205rad，没有旧版几十rad分支。该测试只更改本次代码，未换碰撞模型。

旧版非residualBones100k对照也确认：Pico3关闭viewer后，腿tracking RMSE从.14180降到.08997rad，踝从.12889降到.10083rad，物理时间50Hz采样下踝误差10–24Hz功率从4.0391e−5降到1.1385e−5rad²。这个比例仅对应本条动作/指标，不能当作所有动作的固定收益。
