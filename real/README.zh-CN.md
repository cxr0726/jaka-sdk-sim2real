# KPI 真机部署

[English](README.md)

`kpi_deploy_real` 用于在腿腰式机器人上运行运动策略和有限状态机（FSM）。
硬件通信由随附的 `legged_waist_robot_intf` 库所提供的
`legged_waist_controller::RobotIntf` 负责。

有关接口架构、生命周期、数据流、API 调用映射和运行时要求，请参阅
[RobotIntf 集成说明](docs/robot_intf_integration.zh-CN.md)。

## 部署与运行

以下命令中的 `<控制器用户名>` 和 `<控制器 IP>` 需要替换为实际值。

### 第一步：编译工程

本项目是一个 ROS 2 `ament_cmake` 软件包，目标平台为 ROS 2 Humble。在
`real` 目录下执行：

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build -j
```

生成的可执行文件为 `build/KpiDeployReal`。构建过程使用：

- `ext_libs/legged_waist_controller/include` 中的 API 头文件；
- `runtime_lib` 中的 `liblegged_waist_robot_intf.so` 和 ROS 消息类型支持库；
- `ext_libs/onnxruntime-linux-x64-1.24.4` 中的 ONNX Runtime。

### 第二步：整理并上传部署文件

将可执行文件、`config`、`runtime_lib` 和 `startup.sh` 放到同一个
`sdk_demo` 文件夹中：

```bash
rm -rf deploy/sdk_demo
mkdir -p deploy/sdk_demo
cp build/KpiDeployReal startup.sh deploy/sdk_demo/
cp -r config runtime_lib deploy/sdk_demo/
chmod +x deploy/sdk_demo/KpiDeployReal deploy/sdk_demo/startup.sh
```

整理后的目录结构如下：

```text
deploy/sdk_demo/
├── KpiDeployReal
├── config/
├── runtime_lib/
└── startup.sh
```

使用 `scp` 将整个文件夹上传到控制器的主目录：
无线网:jakauser@10.42.0.1,密码：jaka007
有线网：jakauser@10.5.5.100,密码：jaka007

```bash
scp -r deploy/sdk_demo jakauser@10.42.0.1:~/
```

### 第三步：登录控制器并停止本地 jkpi 服务

远程登录控制器：

```bash
ssh jakauser@10.42.0.1
```

停止控制器上正在运行的本地 `jkpi` 服务，避免它与 `sdk_demo` 同时控制机器人：
另外，重启控制器话题服务

```bash
sudo systemctl stop jkpi-kpideploy.service
sudo systemctl status jkpi-kpideploy.service
sudo systemctl restart jkkhan-ros2.service
```

当状态显示为 `inactive (dead)` 时，再继续下一步。

### 第四步：启动 sdk_demo

进入控制器主目录中的工程文件夹，通过 `startup.sh` 启动：

```bash
cd ~/sdk_demo
./startup.sh
```

`startup.sh` 会选择 Fast DDS、加载 `config/fastdds.xml`，然后启动
`KpiDeployReal`。机器人控制器及其 ROS 接口必须已启动，并且可在同一 ROS 域中被发现。

`ROS_LOCALHOST_ONLY` 默认为 `1`。如果需要与另一台主机通信，请将其设为 `0`，并按需
指定 ROS 域：

```bash
ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=<domain> ./startup.sh
```

在确认关节反馈、EtherCAT 运行状态、IMU 数据、关节顺序、限位和急停行为均正确之前，
请勿使能机器人。
