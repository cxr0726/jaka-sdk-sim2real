# SuperMimic 旧部署路径与目标角限幅对照

2026-11-04，按要求恢复原并联求解算法和原 motion buffer 语义。

- Super 使用独立 `LegacyRealtimeMotionBuffer`，与 real 原接收器等价：本地墙钟插入/查询，首包立即 ready，base_link[0]，无隐式加高、源时间预充或 generation/stale 状态。
- 进入模式首步初始化 yaw/action/history；首包到达再次对齐 yaw，保留动作和状态历史。
- 原 `ParKine` 正逆解、SDK PD 和策略反馈使用同一旧 ABI。共享接口导致其他 sim 模式也恢复旧脚踝算法；普通 ZMQ/fixwrist 的 source-clock buffer 保持原配置。
- 原解算负返回恢复单腿 PR 位置/速度置零；通用非有限值/物理异常隔离保留。
- 模型、关节映射、增益、XML 和物理模型未修改。

## 可选目标角限幅

默认关闭；开启只限制物理 PR 目标，不裁剪 raw action 或 history：

`target = clip(q0 + action_scale * raw_action, SDK lower, SDK upper)`

当前27关节限位按名字与 real 一致；四踝使用 pitch `[-.52,.61]`、roll `[-.35,.35]`。配置项为 `jakaminimimic_superzmq.clip_target_positions`，要求 `angle_map=0`。

```bash
cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
./scripts/run_super_npz_demo.sh scripts/newsmallstep.npz --viewer --duration 0 --target-limits none
./scripts/run_super_npz_demo.sh scripts/newsmallstep.npz --viewer --duration 0 --target-limits real
```

NPZ 默认不加高；显式 `--height-offset` 在发布端统一平移世界 Z 一次，不改变 buffer 时间轴。诊断记录保留 raw action、限幅前后目标和明确的 legacy backend 标记。

## 验证

编译、离线检查及配对闭环动作测试进行中。结果完成后补充此处。测试只连接本地仿真，不连接实机控制器。
