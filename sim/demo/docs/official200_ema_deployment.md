# Official 200k EMA 部署核对与动作测试

2026-11-12。模型为 `config/policy_mimic/ckpt_0200000_bones_official_ema.onnx`。
训练来源为 `SuperMimic-Reproduced-jaka` 的
`logs/khan_8452_official_clip_ln_gelu_4096_new/ckpt_0200000.pt`，显式导出 EMA 权重。
sim 与 real 目录中的同名 ONNX 完全相同，SHA256：

```text
11a86dd003a812e79d31725cee0e6754cdc93b1d3893edd62c9e857f4df47ee6
```

本次不切换 sim/real YAML 的默认策略，不修改 PD、motion buffer、并联求解器、
目标限幅或默认碰撞模型。real 工作区已有的脚踝解算与关联错误处理改动不属于本次提交。

## 接口与必要的加载器适配

输入仍是 float32 `obs[batch,620]`，输出 `actions[batch,27]`；
base_link 锚点、五未来帧、五历史帧、27关节顺序、默认角、action scale、
PD 和训练机器人资产合同均与之前 BONES residual160 / EMA180 一致。

新策略使用 `action_mode=default`，没有参考残差动作。C++ 不需要新的 FSM 或动作公式：

```text
q_raw = q0 + action_scale * raw_ONNX_actions
      = q0 + 0.5 * raw_ONNX_actions
```

不额外加 reference，不对动作再做 EMA；last_action/history 继续保存上次完整 raw action。

原 sim 加载器硬要求 `running_normalization=false`，会拒绝本模型。
现在兼容两种合同：旧模型无运行归一化；新模型必须明确声明归一化在图内、推理不更新，
且公式、sensor scale 逆变换及各统计布局符合约定。没有改变 C++ 的 obs/action 计算。

新 ONNX 图内执行：

```text
x_raw = x_cpp / caller_scale
x_normalized = (x_raw - saved_mean) / (saved_std + 0.01)
```

调用方仍提供 `omega*0.5`、`qd*0.2`，图内分别乘2、乘5恢复物理单位后归一化。
传感器87维统计在五个history lag之间共享，orientation使用独立30维统计，
goal使用独立155维统计。均值/标准差、LayerNorm/GELU及EMA权重已导出，
调用方不另加归一化、逆缩放或在线统计。

real 的 YAML-owned adapter 原本只检查张量接口，不从 metadata 导入关节参数，
因此不需要相应加载器修改。离线核对确认当前 real YAML 的关节顺序、q0、scale、
obs scales 和新模型相符；硬件转换后的 PD 继续从 YAML 读取。

## 数值验证

- 生产 sim adapter 加载新模型及旧100k模型通过；全部27关节映射、默认角、scale、PR PD通过。
- 拒绝缺失归一化合同、非图内归一化、推理更新统计、错误epsilon或sensor逆缩放声明。
- EMA PT/ORT：128随机、128动作参考、全零和动态batch验证通过；最大action误差5.25e-6，target误差2.63e-6rad。
- 独立公式核对实际图中间输出：goal及完整465维policy obs归一化误差0；推理不更新buffer。
- 实际 C++ Pico2 全718tick：训练公式重建obs最大差5.03e-7，reference关节角/history raw action差0，target散射差0，C++/PT action最大差3.27e-6。
- 实际本机50Hz ZMQ Pico3全片完成；冻结其实际消费窗口再运行，1041条完整状态、20820物理子步、1038条policy逐值复现（除查询时间）。
- 用新加载器运行旧180k，同参考的旧状态、action、物理子步逐值复现。
- real adapter 独立加载/映射测试通过；未构造硬件接口。
- 本机 ORT CPU 单线程同1000条obs：新模型median约.39–.40ms，旧EMA180约.38–.39ms。

## C++完整动作结果

使用当前SDK闭链和默认 `Khan_mini_contact.xml`、参考高度0、target limits none。
每个模型独立计算自身history/推理/action/physics，同动作共享完全相同的五帧reference。
排除启动与末端holding，比较原生物理PR角，而非将AB解算反馈当真值。

| 动作 | 踝RMSE：旧160 / 旧180EMA / official200，rad | PR角25–90Hz功率：200相对180 |
|---|---|---:|
| motion_pico2 | .07269 / .06926 / .08565 | +17.1% |
| motion_pico3 | .08950 / .08947 / .09777 | −7.8% |
| dance1_subject2 | .10844 / .10854 / .11524 | +1.4% |
| walk2_subject1 | .10019 / .10184 / .12068 | +0.4% |
| fallAndGetUp2_subject2 | .15554 / .15073 / .16189 | −7.4% |

五个official200 none动作均完整完成；起身动作第一次及本轮18个参考恢复窗口均恢复。
这几个C++动作没有统一改善，高频功率变化不能直接当作肉眼振幅百分比。
旧160第一次起身失配使getup全片指标包含不同运动状态，不能把频谱作为纯抖动优劣排序。

另做原生mjlab Pico3同条件新旧EMA对照：踝RMSE .08558→.09201rad，
normalized return .95193→.94792，均完整完成。Pico3退步也出现在训练侧，
并非仅C++加载/观测适配导致。输入仅补名字并重排关节，原始pose/速度/高度不变。
这里两框架初始化、参考播放相位和执行动力学不同，跨框架绝对差值不是单因素消融。

official200 原生mjlab完整getup第一次正常，normalized return约.80075。
SDK移植训练接触体的none完整getup，腿/踝RMSE由.16123/.16189改善至.14643/.15303，
PR高频功率约下降6.3%；仍不是全动力学对齐。脚STL本来一致，主要移植差异在非足部接触体。

## 脚踝目标限幅

下表是 **none闭环轨迹** 中raw target超过pitch[-.52,.61]、roll±.7的比例。
它是指定轨迹上的反事实裁切比例；加入裁切后反馈变化，新的raw target分布也会改变。

| 动作 | 左pitch | 右pitch | 左roll | 右roll |
|---|---:|---:|---:|---:|
| Pico2 | 0% | .18% | 2.64% | 6.51% |
| Pico3 | 0% | 0% | 2.14% | 4.88% |
| Dance | 2.02% | 3.07% | 5.87% | 7.64% |
| Walk | 1.06% | 1.38% | 9.30% | 4.30% |
| Getup | 2.59% | 2.40% | 2.14% | 3.82% |

新模型没有消除超target。Pico3右roll raw最大约.996rad；Dance左右roll可约±1.3rad。
这些是虚拟PD平衡点，不是实际关节姿态。训练/SDK的物理pose范围仍约
pitch[-.5236,.6108]、roll±.349；MuJoCo采用软约束，实际角可短暂略越range。
训练 `w_joint_limit=-2` 惩罚实际位置，训练没有raw target clip。

独立闭环比较保持其他25轴target bounds相同，仅改roll，pitch固定[-.52,.61]：

| 动作 | roll范围 | 踝RMSE，rad | PR25–90Hz功率，rad² |
|---|---|---:|---:|
| Pico3 | ±.35 / ±.5 / ±.7 | .09603 / .09684 / .09786 | 2.231 / 2.544 / 2.158 e-7 |
| Dance | ±.35 / ±.5 / ±.7 | .12245 / .12064 / .11876 | 4.319 / 4.282 / 3.870 e-7 |

没有证据表明±.5统一更稳；Dance的±.7比±.35跟踪与高频略好。
若沿用目前实机的虚拟target部署规范，可保留 **pitch[-.52,.61]、roll±.7** 为已有基线，
不因raw峰值继续放宽。±.7不是机械roll行程许可。不同PR位置/速度下可达域与AB扭矩耦合，
不能用一个raw峰值给机械限位定值；本轮未重新标定实机机械范围。

同一反馈下，改target会改变PR力矩：`Δtau_PR = Kp * (q_clipped - q_raw)`，
pitch Kp80，roll Kp20，随后经Jᵀ映射与下层限矩。全部本轮最终AB广义执行力
`qfrc_actuator`未达到±96Nm；raw target越界不等于已发生最终AB力矩饱和。

完整getup开启全27轴real目标限幅、roll±.7时，默认SDK及训练contacts均有晚段恢复窗口失配，
而none两组均18/18恢复。全27轴裁切同时涉及其他轴，不能把变化全归于踝。
这也说明常规Pico/Dance通过不保证动态起身在限幅后匹配训练控制律。

**生产sim启动器 `--target-limits real` 仍使用sim现有roll±.35，
不是本报告临时诊断hook的real-YAML±.7。** 这些.7及候选闭环实验未修改生产GlobalVars，
也没有自动调整real YAML。none可用于观察原始策略控制律；实机继续使用自身部署限幅。

## 可视化命令

```bash
cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
./scripts/run_super_npz_demo.sh scripts/motion_pico3.npz \
  --policy config/policy_mimic/ckpt_0200000_bones_official_ema.onnx \
  --viewer --duration 0 --target-limits none --height-offset 0
```

将第一参数替换为`motion_pico2.npz`、`dance1_subject2.npz`、`walk2_subject1.npz`或
`fallAndGetUp2_subject2.npz`（均在`scripts/`内）。发布结束保持末帧，Ctrl-C清理本次进程。
`--policy`只覆盖本次生成的回放配置，mode仍为`jakaminimimic_superzmq`。

若在real侧选用，手动将`real/config/KpiConfig.yaml`中
`jakaminimimic_superzmq.policy_path`指向`config/policy_mimic/ckpt_0200000_bones_official_ema.onnx`；
保持YAML-owned关节顺序、q0、scale、转换PD和PR增益。这次没有替你修改默认路径。

## 本机复核资料

完整记录位于仓库外提交范围的`diagnostics/official200_comparison_20261112/`：

- `contract/REPORT.zh-CN.md`：模型、图内公式、实际C++718tick和实机离线协议验证。
- `motion/MOTION_RUNS.zh-CN.md`、`motion/primary_comparison.json`：20组捕获、同参考新旧指标及真实ZMQ复现。
- `limits/REPORT.zh-CN.md`、`limits/metrics.json`：逐轴raw/actual/裁切/PR力矩变化和最终AB力矩。
- `limits/raw_targets_vs_native_PR.png`：虚拟target和原生物理PR角的区别。
- `mjlab/pico3_comparison.json`：同名输入的训练侧新旧对照。
- `build/production_build.log`、`build/production_ctest.log`：正式demo与协议CTest。

诊断数据、临时碰撞模型和视频不写入git；ONNX副本、最小加载器兼容代码、回归测试与本文纳入提交。
