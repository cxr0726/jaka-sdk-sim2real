录制数据回放与数值审计：20261022-221234（2026-10-30）

已经核对全部10集的Parquet、JSON和motion_raw.npz。原始录制文件未修改。
审计统计位于build/record_replay/data_audit.json，实际联调结果位于同目录live_checks.json。

**数据规模和总体判断。**

Parquet合计10,709行，约29.93–29.94Hz；原始NPZ合计17,908帧，约49.82Hz。
40维actions含有效的遥操作参考；30维state在全部行、全部维度均为零，不能用作真实机器人反馈。
数据可以驱动当前C++的620维跟踪策略，但这不是对机器人稳定性或实机追踪精度的证明。

| episode | Parquet行数 | 原始NPZ帧数 | 表格时长s | 实际表格Hz | 暂停行 | 重复参考行 |
|---|---:|---:|---:|---:|---:|---:|
| episode_0000 | 175 | 301 | 5.812 | 29.9396 | 0 | 0 |
| episode_0001 | 1655 | 2766 | 55.259 | 29.9316 | 0 | 5 |
| episode_0002 | 1558 | 2603 | 52.018 | 29.9318 | 159 | 161 |
| episode_0003 | 367 | 616 | 12.225 | 29.9399 | 0 | 0 |
| episode_0004 | 855 | 1430 | 28.532 | 29.9317 | 0 | 0 |
| episode_0005 | 1352 | 2259 | 45.132 | 29.9344 | 0 | 0 |
| episode_0006 | 1277 | 2133 | 42.625 | 29.9352 | 90 | 96 |
| episode_0007 | 652 | 1091 | 21.748 | 29.9341 | 0 | 0 |
| episode_0008 | 686 | 1148 | 22.881 | 29.9373 | 0 | 0 |
| episode_0009 | 2132 | 3561 | 71.199 | 29.9303 | 233 | 232 |

每集NPZ比Parquet覆盖略早/略晚的一小段。按motion_publish_t_ns精确匹配，除每集第一行
早于该集原始NPZ起点外，其余10,699行都能匹配到相同原始消息，joint_pos逐元素误差为零。
这说明两层记录中的运动内容相容。Parquet以30Hz取最新帧，自然省略部分50Hz源帧。

**数值字段的含义和用途。**

| 列/区间 | 单位与语义 | 对当前C++参考跟踪的用途 |
|---|---|---|
| actions[0:27] | 27个参考关节角，rad，JAKA/MuJoCo顺序 | 直接构造joint_pos；不是ONNX原始action或最终电机扭矩 |
| actions[27:30] | 参考腰部锚点roll、pitch、yaw角速度，rad/rad/s | 当前跟踪器不直接使用；可分析动作、生成其他控制器/VLA标签 |
| actions[30:33] | 锚点局部坐标线速度，m/s | 当前跟踪器不直接使用；可分析速度与重定向质量 |
| actions[33:36] | 参考腰部世界位置xyz，m | 直接构造参考腰部位置 |
| actions[36:40] | 参考腰部世界姿态wxyz，单位四元数 | 直接构造参考腰部姿态，包含绝对yaw |
| state[0:30] | 预期机器人关节和根部状态 | 本数据全零，只能视为缺失反馈/占位，不能算跟踪误差 |
| motion_joint_pos[27] | 原始报文关节角，JAKA顺序 | 与actions前27维完全相同，可作一致性检查 |
| motion_body_pos_w[84] | 28×3完整参考body世界位置 | 参考脚高、支撑关系、FK一致性诊断；reshape后腰在13 |
| motion_body_quat_w[112] | 28×4完整参考body世界姿态，wxyz | 完整骨架姿态诊断；腰在13 |
| motion_seq | 发布序号 | 判断源发布帧顺序；30Hz记录跳1/2个seq通常是正常下采样 |
| motion_paused | 发布端暂停标记 | 区分有意冻结和正常运动；保留暂停时长才能忠实回放 |
| motion_dup_prev | 与前一个表格帧的关节/位置重复 | 诊断静止/暂停/重复发布，单凭重复不能判定头显卡顿 |
| motion_age_ms | 录制tick时间减最近publish时间 | 可观察数据新鲜度，但混合了采样相位、排队、时钟差 |
| idx、head_image_frame_index | 记录行/图像帧索引 | 与视频对齐；本次提供目录没有实际mp4或图像字节 |

动作腰部位置和姿态与motion_body_*的index13逐项匹配。所有actions关节与motion_joint_pos完全一致。
参考roll/pitch与腰部四元数计算值最大差小于8e-8rad；yaw_vel与按实际record tick差分的yaw速率
最大差约4.6e-6rad/s，数值关系自洽。

一个确定的元数据问题：data.json的info.motion_diagnostics.body_names写成Isaac顺序，声称腰在3；
实际motion_body_*是JAKA顺序，腰在13。motion_raw.npz自身存的body_names也确认腰在13。
不能直接依data.json这一段错误标签把body[3]当腰；actions[33:40]和原始数组提供了交叉验证。

**时间戳能否使用。**

1. timestamp_ns是int64 Unix纳秒，属于录制进程30Hz采样tick。十集均严格递增，无重复或倒序，
   相邻间隔全局范围29.915904–35.948032ms，中位约33.40ms。首集开始约北京时间
   2026-10-22 22:12:35，与session名称22:12:34相容。JSON与Parquet时间戳逐项相同。
2. motion_publish_t_ns是该行所选源消息的发布端墙钟。全部逐行递增，适合匹配原始流和区分
   录制tick与源消息时间。motion_age_ms与(timestamp_ns−motion_publish_t_ns)/1e6的最大误差
   小于1e-6ms；age范围0.302–25.877ms，P95约19–20ms，符合30Hz抽样50Hz最新帧的相位范围。
3. motion_raw.npz的publish_t_ns/recv_t_ns是约50Hz完整消息的发布/接收时刻。seq无缺号；
   recv−publish平均约0.43–0.49ms。这个差值仍混有跨机offset和处理延迟，不能单凭它证明真实
   网络单程延迟或时钟同步误差。回放轨迹优先使用publish_t_ns，避免把到达抖动编码进动作。
4. smplx_t_ns在live帧全零；仅暂停帧非零，且非零掩码与paused完全一致。它在这份记录里没有
   提供连续有效的头显采样时钟，不能拿来重建整个episode时间轴。
5. xr_body_ts_ns、pico_recv_t_ns全零，xr_pose_hash全空、xr_is_new全false、xr_stale_count全零。
   这些诊断在本次数据中没有填充，不能用来推断真实头显帧率、数据新旧或头显到主机延迟。
   smplx_same_prev大量为true也可能只是0==0，并非真实头显姿态重复的证据。
6. timestamp_ns全为256的整数倍，与先保存浮点Unix秒再转换为纳秒整数的量化特征一致；在当前epoch
   上精度约亚微秒量级。它足够支持30/50Hz时序，但不应宣称具备真实1ns采集精度。后续录制宜用
   time.time_ns()，耗时和本机播放定时宜用time.monotonic_ns()。

结论是：录制tick与发布/接收时间的数值和顺序自洽，可以用来回放；它们不是已验证准确对时的
Pico硬件采样时间。传感器时钟和两台机器真实同步误差没有足够信息从这批单向记录恢复。

**原发布脚本中已修复的问题。**

- 原脚本只找timestamp，而本数据列名为timestamp_ns，导致静默退回arange(N)/30。现在读取真实
  int64时间，在整数域减首帧后转换为相对秒；首集时长从假设5.800s恢复为真实5.811691s。
- 原motion_raw.npz加载会取body[0]作为anchor，实际应取JAKA body[13]；原始NPZ现在按录制格式
  识别JAKA关节顺序、使用publish_t_ns并取正确腰部。无需读取NumPy2生成的object元数据，兼容
  当前最小环境中的NumPy1.21.5。无标记训练NPZ按Isaac约定，导出记录则携带显式joint_order。
- 插值不再用随意增加0.1ms的方法隐式修补倒序时间，非法时间轴会明确报错。关节/位置为线性插值，
  四元数为SLERP；50Hz采样端点数量取整带来的总时长差不超过约10ms，不是恢复原生50Hz的全部信息。
- --no-interpolate默认改用实际记录帧率；--speed在该模式也会生效。
- 导出保留float64相对秒和原始int64纳秒，不将epoch时间转成float32；差分速度按实际时间计算。
- 发包publish_t_ns始终重新生成当前墙钟。旧的2026-10-22时间只定义动作轨迹，不能直接送入
  以当前墙钟查询的motion buffer。

Parquet回放发送joint_pos与腰部pose就足以驱动当前620维obs；为了当前协议的28-body形状，脚本
将腰部pose复制到28个槽位。这些是anchor占位数据，并非28个真实body的FK，不能据导出NPZ的
body_ang_vel_w（目前为零占位）或复制body_pos来分析脚高。分析全骨架应读原始motion_body_*或motion_raw.npz。

**运行方式。**

    cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
    ./scripts/run_record_demo.sh
    ./scripts/run_record_demo.sh scripts/20261022-221234/level-0/episode_0001
    ./scripts/run_record_demo.sh scripts/20261022-221234/level-0/episode_0001/motion_raw.npz

传整个session目录时选择排序后的第一集。默认50Hz插值、单次回放，结束后C++保持最后参考；
KPI_RECORD_LOOP=1可循环。启动器先等本次C++进入ZMQ状态并发布有效状态，再从源第0帧开始发布。
KPI_RECORD_SPEED控制速度；KPI_RECORD_VIEWER=0可无界面联调。关闭viewer或Ctrl+C会清理本次进程。
生成的localhost配置和日志位于build/record_replay/，不改变原始远程Pico配置。

Python环境复用已有NumPy/PyZMQ；仅补充PyArrow25.0.1和兼容当前NumPy的SciPy1.10.1，无需pandas。

**实际联调结果。**

全部10集已完成加载、严格时间轴、四元数有效性及30→50Hz插值检查。另做以下实际ZMQ/C++联调：

| 用例 | 实际收包稳态 | 结果 |
|---|---|---|
| parquet_ep0000 | 50Hz | 单次完整播完，C++保持末参考，随后按duration正常退出，exit=0 |
| raw_npz_ep0000 | 50Hz | 单次完整播完，C++保持末参考，随后按duration正常退出，exit=0 |
| parquet_ep0003 | 50Hz | 短时接收，按duration正常退出并清理发布端，exit=0 |

三个用例均先收到控制器有效状态再从第0帧发布，控制端保持state=11、err=0。首尾统计不足整秒时
接收Hz会低于50，不能把这些部分窗口计数当作持续掉帧。完整episode_0000的Parquet和原始NPZ均实际播完。

这些检查证明协议、时间读取、插值、发布、消费、单次结束和进程清理可运行。
本次尚未逐集评估完整动作的跟踪稳定性。

此外已用episode_0001/data.parquet打开实际GLFW viewer联调，确认50Hz稳态接收、state=11、err=0；
当前按单次回放运行，播放结束后继续保持末参考，关闭viewer即可结束。

**可选发送 base / waist。**

发布脚本新增 `--anchor waist|base`，默认waist。base模式从Parquet的完整
motion_body_pos_w/motion_body_quat_w中取body[0]；原始NPZ也取body[0]。
waist模式继续使用actions[33:40]，或原始NPZ的正确腰部索引。
选定link的位置和四元数都会复制到28个槽位，供接收端按其约定消费。
`run_record_demo.sh --anchor base`和`--anchor waist`会向发布端传入对应选择。
新导出的占位NPZ记录anchor_body_name和body_pose_format；缺少另一link真实pose时明确报错。
