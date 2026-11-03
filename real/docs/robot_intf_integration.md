# Integration with `legged_waist_robot_intf`

[简体中文](robot_intf_integration.zh-CN.md)

## Purpose and boundary

This project does not communicate with motor drivers directly.
`KpiDeployReal` links against `liblegged_waist_robot_intf.so` and creates one
`legged_waist_controller::RobotIntf` object. `RobotIntf` owns an internal ROS 2
node, subscriptions, publishers, service clients, and (by default) its spin
thread. It translates between the synchronous C++ API used by the FSM and the
ROS interfaces exposed by the running `legged_waist_controller`.

The integration has four layers:

```text
KPI FSM and ONNX policy
        |
        | RobotIntf C++ calls and cached snapshots
        v
liblegged_waist_robot_intf.so (internal ROS 2 node/spin thread)
        |
        | ROS 2 topics and services over DDS
        v
legged_waist_controller -> EtherCAT/drives and sensors
```

The vendored public API is
`ext_libs/legged_waist_controller/include/legged_waist_controller/usr_intf/robot_intf.h`.
`include/RLController.hpp` includes its forwarding header, `robot_intf.hpp`.

## Build-time connection

`CMakeLists.txt` adds the RobotIntf include directory and links
`legged_waist_robot_intf`. The shared object is bundled in `runtime_lib`;
there is no source implementation of RobotIntf in this repository.
Consequently, the header and shared object must be kept at compatible versions.
The shared object also needs the bundled
`legged_waist_controller` and `voice_player` ROS type-support libraries at
runtime.

The `KPI_BUILD_REAL` CMake option controls creation of `KpiDeployReal` and is
`ON` by default.

## Object ownership and lifecycle

The process-wide object is declared as `RLController::mRobot` and owned by a
`std::unique_ptr`.

1. `main()` initializes the ROS context with automatic signal shutdown
   disabled.
2. It loads `config/KpiConfig.yaml`.
3. It constructs `RobotIntf::Options`, enables the Joy subscription, and
   constructs `RobotIntf`.
4. `Options::auto_start_spin` defaults to `true`, so construction starts the
   interface's internal executor thread. If that option is changed to `false`,
   this project calls `start()` explicitly.
5. The 20 ms FSM thread reads RobotIntf snapshots and sends commands.
6. On SIGINT or SIGTERM, the FSM and logging threads stop and join.
7. The process requests `do_enable(false)`, waits 500 ms, calls
   `RobotIntf::stop()`, shuts down ROS, and finally destroys the object.

The ordering matters: RobotIntf is stopped while the ROS context is still
valid. The 500 ms delay is intended to give the disable request time to reach
the controller; it is not an acknowledgement check.

## Runtime ROS configuration

This project currently uses the default `RobotIntf::Options` except for
explicitly enabling Joy. Relevant defaults are:

| Option | Default used here | Role |
| --- | --- | --- |
| `controller_node_name` | `legged_waist_controller` | Target controller parameter node |
| `io_prefix` | empty | Places controller I/O at the root namespace |
| `node_base_name` | `legged_waist_robot_intf` | Internal RobotIntf node name |
| `auto_start_spin` | `true` | Runs the interface executor internally |
| `service_timeout` | 3000 ms | Default wait for services |
| `imu_topic` | `/imu/data` | IMU input cached by RobotIntf |
| `joy_topic` | `/joy` | Gamepad input cached by RobotIntf |
| `joy_timeout` | 500 ms | Marks stale gamepad data offline |
| `control_source` | `JOY` | Initial command source |
| `loco_move_topic` | `/loco_move` | Optional application motion input |
| `pi_state_topic` | `/PiState` | Publishes the current control state |

RobotIntf also creates optional services using its defaults, including
`set_robot_mot_type`, `go_home`, control-source services, recording services,
and basic-configuration services. Names that are not documented as complete
topic names are combined with `io_prefix`. Refer to `RobotIntf::Options` in the
vendored header for the complete list.

`startup.sh` defaults to Fast DDS and `ROS_LOCALHOST_ONLY=1`. The controller
must use a compatible RMW implementation, the same `ROS_DOMAIN_ID`, and a
reachable DDS network. Set `ROS_LOCALHOST_ONLY=0` if the controller is on a
different host.

## Data and command flow

RobotIntf's internal executor updates thread-safe cached state. The FSM's 20 ms
thread polls those snapshots:

| RobotIntf API | Used by this project for |
| --- | --- |
| `get_joint_status()` | Position command/feedback, velocity, enable/fault state, and EtherCAT OP readiness |
| `has_imu()` / `get_imu()` | Startup readiness, policy observations, and CSV logging |
| `get_control_source()` | Deciding whether Joy input should drive state changes |
| `get_joy_robot_state()` | Enable/disable, state transitions, reset, and velocity commands |
| `set_joint_trajectory()` | Publishing ordered position targets from the idle planner or ONNX policy |
| `do_enable()` | Enabling in IDLE and disabling in IDLE, FAULT, and shutdown |
| `clear_error()` | Requesting a drive fault reset from FAULT |
| gain/limit/mode APIs | Configuring drives during INIT-to-IDLE transition |
| `set_cur_control_stat()` | Publishing the FSM state through RobotIntf/PiState |

The ordering in `config/KpiConfig.yaml:joint_names` is the contract between
joint feedback, the ONNX action vector, gains, limits, and outgoing trajectory
positions. All corresponding arrays must have `num_actions` entries in exactly
that order.

## FSM interaction

### INIT

INIT calls `get_joint_status()` for every configured joint until all report
`ethercat_operational=true`. It also waits for `has_imu()`. Before entering
IDLE it configures joint angle limits, selects MIT operation mode, writes MIT
gains, configures PD behavior, and seeds the local trajectory filters from
feedback.

`set_joint_operation_mode()` applies to all joints even though the current code
invokes it inside the per-joint gain loop.

### IDLE

IDLE reads the selected control source and the latest Joy snapshot. Gamepad
BACK requests `do_enable(true)` and A requests `do_enable(false)`. Once every
joint reports enabled, D-pad targets are filtered and sent using
`set_joint_trajectory(joint_names, point)`. LB+Y enters LOCO.

### LOCO

LOCO reads joint and IMU snapshots, converts Joy axes to a normalized locomotion
command, builds the policy observation, runs the ONNX model, clamps actions to
the configured limits, and sends one ordered trajectory point every 20 ms.
A joint becoming disabled or faulted causes transition toward FAULT.

### FAULT

Entering FAULT immediately requests `do_enable(false)`. START calls
`clear_error()` while joint faults remain; once they are clear, START returns
the FSM to INIT so readiness is checked again.

## Return values and stale data

Most RobotIntf commands return an integer status, and
`get_joint_status()` distinguishes an unknown joint from a joint that is not
operational. The current control code generally does not inspect these return
values. It instead observes subsequent cached joint state. For changes that
need immediate confirmation, check and handle the API return value explicitly.

`get_imu()` returns `false` until an IMU sample has arrived.
`get_joy_robot_state()` returns `false` and clears its output when Joy is stale
past `joy_timeout`. Callers should use the boolean result when adding controls
whose stale value could be unsafe.

## Adding another RobotIntf call

When extending the integration:

1. Configure names and optional features through `RobotIntf::Options` before
   construction.
2. Keep one shared RobotIntf instance; do not add a second internal ROS node
   merely to access another API.
3. Use the configured joint-name order for trajectory vectors.
4. Avoid blocking service calls in the 20 ms path where possible; perform
   configuration during state transitions or use the asynchronous API.
5. Handle return codes and define a safe state for missing or stale data.
6. Stop interface-owned threads before `rclcpp::shutdown()`.

For the authoritative signatures and option defaults, consult the vendored
`robot_intf.h`; wire-level implementation details remain inside the bundled
shared library.
