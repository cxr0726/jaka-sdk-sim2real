# 并联踝解算和 CAD 接触模型

修改前快照为 `2195143`。本更新不修改策略权重、关节顺序、观测缩放或实时参考时钟。

## 解算和错误传播

`AB2PitchRoll` 保留原 C ABI，成功返回非负数，失败返回负数。数值实现保留原机构几何和解析速度 Jacobian，增加受限 Newton 步、残差下降回溯、装配分支和最终解的有效性检查。它不再通过修改输入电机角来掩盖测量越界。

装配分支范围用于排除翻转或整圈数学根，不替代 XML 的物理关节限制，也不意味着这些范围已经完成实机标定。

观测侧对左右踝一起提交转换结果。一侧失败时，两侧输入数组均不被部分改写，策略不继续推理，不提交新目标；错误锁存为 `ERR_JOINT`，PD 和驱动使能关闭。上一有效解仅可作为后续求解的初值，不作为无限期替代当前反馈的数据。

`PitchRoll2ABChecked` 是显式返回状态的逆解。`angle_map=1` 的不可达目标不再静默夹紧到 asin 边界；当前 `angle_map=0` 路径仍使用训练定义的 raw PR 伺服目标，不因此新增动作裁剪。

当前交付 SDK 是预编译库，其内部正解调用未检查返回值。`CheckedMujocoPhysics` 因而在每次 1 ms 物理子步之前，以同一套正解和同一份原始电机状态预检；失败时清零控制输入、关闭 PD/使能并停止推进物理，避免无效结果进入 SDK PD。可执行文件显式导出修复后的 C 符号，使 SDK 内部调用与预检使用相同的数值实现。

该保护覆盖本 demo 的全部物理子步。单独调用 SDK 库的其他程序或实机后端仍需接入相同的有效性与错误传播机制。本更新提供显式故障停控，不提供机器人故障后的自动站立恢复策略。

## 默认接触模型

`sdk/assets/khan_mini/Khan_mini_contact.xml` 是基于现有 CAD 的外包络近似，供几何核对，并非已完成材质和受载接触标定的模型。

- 补齐 base、小腿和踝 pitch 外部结构的接触。
- 脚保留带脚背的整脚 mesh，MuJoCo 使用其凸包进行碰撞；visual 凹槽与凸包填充空间并不相同。
- 小腿分为主杆与末端关节包络，避免一整块凸包填满叉架空隙。只屏蔽末端包络与同侧脚的内部机械配合；主杆与同侧脚、异侧脚与腿、左右脚之间的碰撞保留。
- base 与同侧髋 roll 的机械内部包络重叠使用局部排除，不关闭 base 与地面、腿或脚的接触。
- 保留原质量、惯量、关节限制、并联约束、armature、PD 和接触材料参数。

生成与检查脚本为 `scripts/generate_contact_model.py` 和 `scripts/validate_contact_model.py`。原始 `Khan_mini_sym.xml` 保留。

新增几何与现有 ONNX 所记录的训练接触模型不同。旧策略可能仍出现跟踪质量下降，尤其是下跪、起身和脚间净空较小的动作。截图用于人工核对；后续训练应采用同一接触定义并重新适配，不能把可视化启动成功视作实机验证。

## 指定 NPZ，带 viewer 播放

在 `sim/demo` 下运行：

```bash
./scripts/run_super_npz_demo.sh scripts/walk2_subject1.npz --viewer --duration 0
./scripts/run_super_npz_demo.sh scripts/dance1_subject2.npz --viewer --duration 0
./scripts/run_super_npz_demo.sh scripts/fallAndGetUp2_subject2.npz --viewer --duration 0
```

每次启动一条命令。控制器先进入可用状态，再从动作开头发布。`--duration 0` 持续运行，NPZ 结束后保持末帧；关闭 viewer 或 Ctrl-C 结束。具名训练 NPZ 的高度补偿为 0，Pico 原有补偿配置保留。

对比旧接触几何时，仍使用新解算器：

```bash
./scripts/run_super_npz_demo.sh scripts/walk2_subject1.npz --viewer --duration 0 \
  --xml ../sdk/assets/khan_mini/Khan_mini_sym.xml
```

实时 Pico 控制：

```bash
./scripts/run_demo.sh --viewer --duration 0 --mode jakaminimimic_superzmq
```

## 检查

```bash
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

本次全部 9 项 CTest 通过。新增检查覆盖 2601 个 PR 姿态、5 个已记录的旧版错误根、解析 Jacobian 有限差分、非法输入/不可达逆解、观测交易式提交、PD 子步前故障隔离，以及接触掩码和未改变的动力学参数。真实 SDK 集成测试还验证左右踝的实际 actuator 控制输入与新解的 `Jᵀ·τ_PR` 一致，确认修复后的符号确实被预编译 SDK 调用。

## 本次动态验证（2026-11-02）

使用同一现有 SuperMimic ONNX、新 CAD 接触 XML、实时 ZMQ NPZ 发布、`angle_map=0`。启动器先等待控制器与有效状态，再发布第 0 帧。以下时间是包含启动和末帧保持的仿真运行时间；步行仅测试开头 30 秒，舞蹈与起身已覆盖完整原始片段。

| 动作 | 运行时间 | 数值/错误状态 | 结束时 base 高度 / 倾角 |
| --- | ---: | --- | --- |
| walk2_subject1 | 30 s | 全部有限，`err=0` | 0.627 m / 3.58° |
| dance1_subject2 | 138 s | 全部有限，`err=0` | 0.637 m / 6.58° |
| fallAndGetUp2_subject2 | 170 s | 全部有限，`err=0` | 0.170 m / 102.37°，仍倒地 |

起身动作未完成；数值稳定和正常退出不等于成功跟踪动作。新接触定义与旧策略训练条件存在差异；本次保留外部碰撞、SuperMimic 的原始动作语义和策略权重。

viewer 成功加载新模型并接入动作，但本次桌面处于锁屏状态（GNOME ScreenSaver `GetActive=true`），图形调用严重限速，因此 viewer 试运行只确认加载与接入，未作为实时动作效果验证。动态结果来自无 viewer 的实时运行。用户查看时应在已解锁桌面运行上述命令。

完整 CSV、日志和 `runtime_summary.json` 位于工作区 `diagnostics/production_fix_20261102/`。微基准中正常与已记录坏根输入平均单次正解约为 0.14–0.34 μs，仅代表本机典型求解器耗时，不包括策略、物理或渲染，也不是最坏耗时上界。
