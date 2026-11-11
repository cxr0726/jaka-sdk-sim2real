# KPI real-robot deployment

[简体中文](README.zh-CN.md)

2026-11-11: SuperMimic now selects `ckpt_0160000_bones_residual.onnx`.
This real release retains the legacy ankle solver, error handling and motion
buffer. Reference skip is embedded in ONNX; control parameters remain YAML-owned.
See the [residual160k deployment notes](docs/residual_160k_deployment.zh-CN.md).

Khan SuperMimic real deployment uses YAML-owned joint parameters and hardware
gains. See the [migration and validation notes](docs/supermimic_deployment.zh-CN.md)
for the LB+B entry/exit gesture, legacy receiver-arrival buffer and offline checks.

`kpi_deploy_real` runs the locomotion policy and finite-state machine (FSM) for
the legged-waist robot. Hardware communication is delegated to
`legged_waist_controller::RobotIntf`, supplied by the bundled
`legged_waist_robot_intf` library.

For the interface architecture, lifecycle, data flow, API call map, and runtime
requirements, see [RobotIntf integration](docs/robot_intf_integration.md).

## Deployment and operation

Replace any placeholders such as `<domain>` with the actual value for your
environment.

### Step 1: Build the project

The project is a ROS 2 `ament_cmake` package and targets ROS 2 Humble. Run the
following commands from the `real` directory:

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build -j
```

The executable is `build/KpiDeployReal`. The build consumes:

- API headers from `ext_libs/legged_waist_controller/include`;
- `liblegged_waist_robot_intf.so` and ROS message type-support libraries from
  `runtime_lib`;
- ONNX Runtime from `ext_libs/onnxruntime-linux-x64-1.24.4`.

### Step 2: Prepare and upload the deployment files

Place the executable, `config`, `runtime_lib`, and `startup.sh` together in an
`sdk_demo` directory:

```bash
rm -rf deploy/sdk_demo
mkdir -p deploy/sdk_demo
cp build/KpiDeployReal startup.sh deploy/sdk_demo/
cp -r config runtime_lib deploy/sdk_demo/
chmod +x deploy/sdk_demo/KpiDeployReal deploy/sdk_demo/startup.sh
```

The resulting directory layout should be:

```text
deploy/sdk_demo/
├── KpiDeployReal
├── config/
├── runtime_lib/
└── startup.sh
```

Upload the entire directory to the controller's home directory with `scp`.
The controller connection details are:

- Wi-Fi: `jakauser@10.42.0.1`
- Wired network: `jakauser@10.5.5.100`
- Password: `jaka007`

The following example uses the Wi-Fi address:

```bash
scp -r deploy/sdk_demo jakauser@10.42.0.1:~/
```

### Step 3: Log in to the controller and stop the local jkpi service

Log in to the controller:

```bash
ssh jakauser@10.42.0.1
```

Stop the controller's local `jkpi` service to prevent it from controlling the
robot at the same time as `sdk_demo`. Then restart the controller's ROS 2 topic
service:

```bash
sudo systemctl stop jkpi-kpideploy.service
sudo systemctl status jkpi-kpideploy.service
sudo systemctl restart jkkhan-ros2.service
```

Continue only after `jkpi-kpideploy.service` is shown as `inactive (dead)`.

### Step 4: Start sdk_demo

Enter the project directory under the controller's home directory and start it
with `startup.sh`:

```bash
cd ~/sdk_demo
./startup.sh
```

`startup.sh` selects Fast DDS, loads `config/fastdds.xml`, and starts
`KpiDeployReal`. The robot controller and its ROS interfaces must already be
running and discoverable on the same ROS domain.

By default, `ROS_LOCALHOST_ONLY=1`. Set it to `0` when communication with
another host is required, and specify the ROS domain as needed:

```bash
ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=<domain> ./startup.sh
```

Do not enable the robot until joint feedback, EtherCAT operational state, IMU
data, joint ordering, limits, and emergency-stop behavior have been verified.
