#pragma once
#include <stdint.h>
#include <cstdint>
#include <chrono>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

// 需要在头文件里使用 JointTrajectoryPoint 作为参数类型（Humble 下它是 using 别名，无法用 struct 前置声明）
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robot_mot_type.h"

namespace legged_waist_controller {

class RobotImpl;

class RobotIntf
{
public:
    /// CiA402 / drive command targets for set_joint_operation_mode()
    static constexpr std::int8_t JOINT_OPERATION_MODE_CSP = 8;
    static constexpr std::int8_t JOINT_OPERATION_MODE_MIT = 12;

    enum class ControlSource : uint8_t
    {
        KEY = 0,
        JOY = 1,    // JOY控制源时，接收joy话题
        APP = 2,    
    };
    struct Safety
    {
        bool is_estop;
    };
    struct Battery
    {
        double voltage{0};
        double current{0};
        double soc{0};
    };
    struct JointStatus
    {
        std::string name;
        uint8_t slave_id{0};
        double pos_cmd{0};
        double pos_fdb{0};
        double vel_fdb{0};
        double effort_fdb{0};
        bool is_enable{false};
        bool is_fault{false};
        uint64_t error_code{0};
        /// EtherCAT 从站是否处于 OP；与 `RobotJointStatus.ethercat_operational` 一致（无该 state 接口时上游可填 false）
        bool ethercat_operational{false};
    };
    struct JointGroup
    {
        std::string name;
        std::vector<JointStatus> joints;
    };
    struct RobotStatusData
    {
        bool is_estop{false};
        JointGroup left_arms;
        JointGroup right_arms;
        JointGroup left_legs;
        JointGroup right_legs;
        JointGroup waists;
        JointGroup heads;
        Battery battery;
        Safety safety;
    };

    /// Matches `sensor_msgs/msg/Joy`: axes (float32[]) and buttons (int32[]).
    struct JoyStickState
    {
        int32_t stamp_sec{0};
        uint32_t stamp_nanosec{0};
        std::string frame_id;
        std::vector<float> axes;
        std::vector<int32_t> buttons;
        float axes_L_X{0};  // 左摇杆
        float axes_L_Y{0};  // 左摇杆
        float axes_R_X{0};  // 右摇杆
        float axes_R_Y{0};  // 右摇杆
        float axes_LT{0};
        float axes_RT{0};
        float axes_DX{0};   // 方向键左右
        float axes_DY{0};   // 方向键上下
        bool buttons_A{false};
        bool buttons_B{false};
        bool buttons_X{false};
        bool buttons_Y{false};
        bool buttons_LB{false};
        bool buttons_RB{false};
        bool buttons_BACK{false};
        bool buttons_START{false};
    };

    /// Latest gamepad sample from `/joy` (sensor_msgs/Joy).
    struct JoyRobotState
    {
        bool joy_online{false};
        JoyStickState joy;
    };

    struct ImuData
    {
        // time in ROS clock (same as incoming Imu header.stamp)
        int32_t stamp_sec{0};
        uint32_t stamp_nanosec{0};

        // frame id (incoming Imu header.frame_id)
        std::string frame_id;

        // orientation quaternion (x,y,z,w)
        double qx{0}, qy{0}, qz{0}, qw{1};
        // angular velocity (rad/s)
        double wx{0}, wy{0}, wz{0};
        // linear acceleration (m/s^2)
        double ax{0}, ay{0}, az{0};
    };

    struct LocoMoveCmd
    {
        // range: [-1, 1]
        double forward{0};
        double lateral{0};
        double yaw{0};
    };

    struct BasicConfigs
    {
        std::vector<double> home_positions;  // 零位
        double max_walking_speed{100.0};     // 最大步行速度, mm/s
    };

    struct OperationCodeSet
    {
        std::string name;
        uint32_t code;
    };
    // 模式A下支持的操作码集合
    using MoveTypeMap = std::unordered_map<uint32_t, std::vector<OperationCodeSet>>;    // key: move_type, value: operation_code_set

public:
    struct Options
    {
        /// ROS namespace for the internal node (e.g. /JkPi_012345). Empty + auto_ros_namespace=true → read env/identity.
        std::string ros_namespace{""};
        /// When true and ros_namespace empty, load from JKKHAN_ROS_NAMESPACE / robot_identity.json.
        bool auto_ros_namespace{true};
        /// 控制器节点名（用于参数服务：/name/set_parameters 等）
        std::string controller_node_name{"legged_waist_controller"};
        /// IO 前缀（空表示相对名，随 ros_namespace 解析）
        std::string io_prefix{""};
        /// 本库内部 `rclcpp::Node` 名（不含命名空间）；空则使用默认 `legged_waist_robot_intf`
        std::string node_base_name{""};
        /// 为 true 时在 `node_base_name`（或默认名）后追加 `_<pid>`，减轻与长存活对端（如仅重启本进程）在 DDS 层的混淆概率
        bool append_pid_to_node_name{false};
        /// IMU 话题（相对名 imu/data，随 ros_namespace 解析）
        std::string imu_topic{"imu/data"};
        /// 是否在构造时自动启动内部 spin 线程（推荐 true，库会自行管理线程安全）
        bool auto_start_spin{true};
        /// 在 `start()` 内、executor 已进入 `spin()` 之后做一次只读 RPC 预热（`get_joint_id_map`）。默认 false；
        /// 若在「刚 spawn spin 线程立刻发 async service」的组合下曾出现崩溃，请保持 false 或改在业务里显式调用 `warm_up_services()`。
        bool warm_up_services_after_start{true};
        /// 等待服务可用的默认超时
        std::chrono::milliseconds service_timeout{3000};

        /***************按键扫描（可选功能）*****************************/
        /// 是否启用按键扫描线程（从 STDIN 读取，适合在前台终端运行的工具程序）
        bool enable_key_scan{false};
        /// 按键扫描周期（ms）
        int key_scan_period_ms{20};
        /// 按键->运动类型映射的配置文件路径（预留，当前版本不解析，保持为空 map）
        std::string key_motion_map_config{""};
        /***************************************************************/

        /***************RobotMotType Service（可选功能）*****************************/
        /// 是否对外提供设置 RobotMotType 的 ROS service（由 RobotIntf 内部 node 提供）
        bool enable_mot_type_service{true};
        /// service 名称（会拼接 io_prefix；例如 io_prefix="/", service="set_robot_mot_type" => "/set_robot_mot_type"）
        std::string mot_type_service_name{"set_robot_mot_type"};
        /**********************************************************************/

        /***************GoHome Service（可选功能）*****************************/
        /// 是否对外提供“回home”的 ROS service（回调中会设置 mot_type=MOT_TYPE_GO_HOME）
        bool enable_go_home_service{true};
        /// service 名称（会拼接 io_prefix；默认 "/go_home"）
        std::string go_home_service_name{"go_home"};
        /**********************************************************************/

        /***************loco_move topic（可选功能）*****************************/
        /// 订阅上层速度指令 topic
        bool enable_loco_move_sub{true};
        /// 话题名（会拼接 io_prefix；默认 "/loco_move"）
        std::string loco_move_topic{"loco_move"};
        /**********************************************************************/

        /***************PiState topic（当前控制状态）*****************************/
        /// 是否发布 PiState（含 cur_control_stat）
        bool enable_pi_state_pub{true};
        /// 话题名（会拼接 io_prefix；默认 "/PiState"）
        std::string pi_state_topic{"PiState"};
        /**********************************************************************/

        /***************Joy（sensor_msgs/Joy，`joy` 包 joy_node 默认发布 /joy）*****/
        /// 订阅手柄话题并缓存 Joy；关闭时不创建订阅与 watchdog 定时器
        bool enable_joy_sub{true};
        /// 完整话题名（相对名 joy，与 joy_linux remap 一致）
        std::string joy_topic{"joy"};
        /// 超过该时间未收到新的 Joy 消息则视为掉线并清空快照
        std::chrono::milliseconds joy_timeout{500};
        /// js 设备路径，用于运行时读取 USB VID/PID 选择按键 layout（与 joy_linux `dev` 一致）
        std::string joy_dev{"/dev/input/js0"};
        /**********************************************************************/

        /***************Control source（KEY / JOY / APP）*****************************/
        /// 默认 JOY（手柄）；APP 为应用 loco_move；KEY 为终端按键扫描
        ControlSource control_source{ControlSource::JOY};
        bool enable_set_control_source_service{true};
        std::string set_control_source_service_name{"set_control_source"};
        bool enable_get_control_source_service{true};
        std::string get_control_source_service_name{"get_control_source"};
        bool enable_control_source_switch_voice{true};
        /*************************************************************************/

        /***************RobotJointGroups record（可选功能）*****************************/
        /// 是否记录 /robot_joint_groups 数据到本地文件（默认关闭）
        bool enable_record_robot_groups{false};
        /// 记录文件路径（CSV）。为空则使用默认路径：/tmp/robot_joint_groups.csv
        std::string record_robot_groups_path{""};
        /***************************************************************************/

        /***************IMU record（可选功能）*****************************/
        /// 是否记录 IMU 数据到本地文件（默认关闭）
        bool enable_record_imu{false};
        /// 记录文件路径（CSV）。为空则使用默认路径：/tmp/imu.csv
        std::string record_imu_path{""};
        /***************************************************************/

        /***************Record activation (shared)*****************************/
        /// 统一激活开关：打开后，会同时启用 robotgroups + imu 两类记录（默认关闭）
        bool enable_record{false};
        /// 是否对外提供录制开关的 ROS service（std_srvs/SetBool）
        bool enable_record_service{true};
        /// service 名称（会拼接 io_prefix；默认 "enable_record"）
        std::string record_service_name{"enable_record"};
        /**********************************************************************/

        /***************BasicConfigs INI（可选功能）*****************************/
        /// RobotIntf 初始化时读取的 ini 配置文件路径；为空则跳过
        std::string basic_config_ini_path{""};
        /**********************************************************************/

        /***************BasicConfigs Service（可选功能）*************************/
        bool enable_basic_config_services{true};
        std::string set_home_positions_service_name{"set_home_positions"};
        std::string set_max_walking_speed_service_name{"set_max_walking_speed"};
        /**********************************************************************/

        /***************MoveTypeMap（模式A 状态机切换表）*************************/
        /// 启动时加载的 ini 路径；为空则使用构建默认路径
        std::string move_type_map_ini_path{""};
        /// 是否对外提供查询切换表的 ROS service（设置仅 C++ API，无 ROS service）
        bool enable_get_move_type_map_service{true};
        std::string get_move_type_map_service_name{"get_move_type_map"};
        /**********************************************************************/

        /***************离线语音（可选）*****************************************/
        bool enable_mot_type_switch_voice{true};
        /// true：拼接「前缀 + digit_0..9」（如 51→「五」「一」逐位）；需 voice_player 生成 digit_*.wav
        bool mot_type_voice_say_number{true};
        /// true：优先播 mot/*.wav 中文名（行走/回零等）；文件缺失时回退 num_XXX
        bool mot_type_voice_say_name{false};
        std::string voice_control_service{"voice_control"};
        /// 默认语音片段目录；为空则用 voice_player package share/audio
        std::string voice_clips_dir{"/usr/etc/jkkhan/share/voice_player/audio"};
        int voice_volume_percent{100};
        /// 仅限制自动触发语音（模式/控制源切换）的最小间隔；play_voice_* 不受此限制
        int voice_min_interval_ms{0};
        /**********************************************************************/

        /***************MotCommand 读取策略（RL 期望指令）************************/
        /// KeepLatest：重复 poll 返回同一指令；ConsumeLatest：poll 后清空；Queue：FIFO pop
        MotCommandPolicy mot_command_policy{MotCommandPolicy::ConsumeLatest};
        /// 仅 Queue 模式：队列满时 drop-oldest
        size_t mot_command_queue_max_size{32};
        /**********************************************************************/

        /***************外部头/爪 JointTrajectory 指令缓存************************/
        /// 相对话题名（经 io_prefix + namespace）
        std::string ext_joint_command_topic{"ext_joint_command"};
        /// 相对 service 名：std_srvs/SetBool，激活写缓存
        std::string ext_joint_command_service_name{"enable_ext_joint_command"};
        /// 白名单；爪子关节名后续追加，不改 API
        std::vector<std::string> ext_joint_command_allowed_joints{"neck_yaw_joint", "neck_pitch_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint"};
        /**********************************************************************/
    };

    RobotIntf();
    explicit RobotIntf(const Options& opt);
    ~RobotIntf();

    RobotIntf(const RobotIntf&) = delete;
    RobotIntf& operator=(const RobotIntf&) = delete;
    RobotIntf(RobotIntf&&) noexcept;
    RobotIntf& operator=(RobotIntf&&) noexcept;

    /// 启动/停止内部 ROS spin（当 auto_start_spin=false 时由用户显式调用）
    int start();
    int stop();

    /// 预热与控制器之间的 RPC 路径：当前实现为拉取一次 `get_joint_id_map`（与 `ensure_id_map` 同源，会填充内部 id 缓存）。
    /// 在 `start()` 之后、对延迟敏感的业务前调用；不替代现场安全确认。
    int warm_up_services();

    /***************操作设置*****************************/
    // 操作指令
    int do_enable(bool enable);
    std::future<int> do_enable_async(bool enable);
    /// 全体关节清零
    int clear_zero_joints(bool request_clear = true);
    std::future<int> clear_zero_joints_async(bool request_clear = true);
    /// 全体关节清除错误（reset_fault 脉冲）
    int clear_error();
    std::future<int> clear_error_async();
    // 单关节清零
    int do_home(uint8_t by_slave_id);
    int do_home(const std::string& by_name);
    /// PD 总开关
    int enable_pd(bool enable);
    std::future<int> enable_pd_async(bool enable);
    /****************************************************/

    /// ESCB 电源操作（对应 /power_operation: std_srvs/SetBool）
    int power_operation(bool enable);
    std::future<int> power_operation_async(bool enable);

    /***************参数设置*****************************/
    /// 设置PD参数
    int set_joint_pd_para(uint8_t by_slave_id, double p, double d);
    int set_joint_pd_para(const std::string& by_name, double p, double d);
    std::future<int> set_joint_pd_para_async(const std::string& by_name, double p, double d);
    /// 关节软限位
    int set_joint_angle_limit(const std::string& joint_name, double min_angle, double max_angle);
    /// 指令/反馈方向
    int set_joint_command_direction(const std::string& joint_name, int direction);
    int set_joint_feedback_direction(const std::string& joint_name, int direction);
    /// 通用 JointCommand 服务
    int joint_command_srv(const std::string& joint_name, uint32_t command, const std::vector<uint8_t>& data = {});
    std::future<int> joint_command_srv_async(const std::string& joint_name, uint32_t command, std::vector<uint8_t> data = {});
    /// 设置运行模式（service: set_joint_operation_mode），全关节共用。mode 为 JOINT_OPERATION_MODE_CSP 或 JOINT_OPERATION_MODE_MIT。
    int set_joint_operation_mode(std::int8_t mode);
    std::future<int> set_joint_operation_mode_async(std::int8_t mode);
    /// 设置单关节 MIT Kp/Kd（service: set_joint_mit_gains；PDO 整型量程见驱动手册）
    int set_joint_mit_gains(uint8_t by_slave_id, uint16_t mit_kp, uint16_t mit_kd);
    int set_joint_mit_gains(const std::string& joint_name, uint16_t mit_kp, uint16_t mit_kd);
    std::future<int> set_joint_mit_gains_async(const std::string& joint_name, uint16_t mit_kp, uint16_t mit_kd);
    /// 设置 joint_command 话题指令的全局插值步数（service: set_interp_steps；必须 >= 1）
    /// 返回 0 成功；-1 服务不可用；-2 超时；-3 服务端拒绝（如 steps < 1）
    int set_interp_steps(int total_steps);
    std::future<int> set_interp_steps_async(int total_steps);
    /****************************************************/

    // 状态获取接口
    RobotStatusData get_robot_status() const;
    /// 返回 -1； 找不到对应id和by_name; 返回： -2； 关节未OP
    int get_joint_status(uint8_t by_slave_id, JointStatus& status) const;
    int get_joint_status(const std::string& by_name, JointStatus& status) const;
    Battery get_battery_state() const;
    bool is_estop() const;

    /***************IMU 数据接口**************/
    /// 是否已经收到过 IMU 数据
    bool has_imu() const;
    /// 获取最新 IMU 数据；若尚未收到过则返回 false
    bool get_imu(ImuData& out) const;
    /// 收到新的 IMU 数据时的回调（可选，在订阅回调线程中调用）
    using ImuCallback = std::function<void(const ImuData&)>;
    void set_imu_callback(ImuCallback cb);
    /****************************************/

    /***************Joy 快照***************************************************/
    /// 返回当前手柄是否在线（在 Options::joy_timeout 时间内收到过 Joy）。
    /// 掉线或从未收到 Joy 时清空 joy 字段并返回 false。
    bool get_joy_robot_state(JoyRobotState& out) const;
    /**********************************************************************/

    // 期望运行指令
    /// 由RL发布单点轨迹，用最新反馈顺序作为 joint list
    int set_joint_trajectory(const trajectory_msgs::msg::JointTrajectoryPoint& traj);
    std::future<int> set_joint_trajectory_async(const trajectory_msgs::msg::JointTrajectoryPoint& traj);

    /// 显式指定关节顺序发布， 需一一对应
    int set_joint_trajectory(const std::vector<std::string>& joint_names, const trajectory_msgs::msg::JointTrajectoryPoint& traj);
    std::future<int> set_joint_trajectory_async(std::vector<std::string> joint_names, const trajectory_msgs::msg::JointTrajectoryPoint& traj);

    /// 订阅到新的机器人状态时的回调（可选）
    using StatusCallback = std::function<void(const RobotStatusData&)>;
    void set_status_callback(StatusCallback cb);

    /// 按 Options::mot_command_policy 取一条 Mot 指令；无指令时返回 false（out.seq_id==0）
    bool poll_mot_command(MotCommand& out);
    /// 只读 peek，不消耗；Queue 模式恒返回 false
    bool peek_mot_command(MotCommand& out) const;

    /***************模式 A 当前控制状态（状态机位置）*************************/
    /// 由上层在「已进入某模式」时调用，作为切换校验的 from 状态（默认 MOT_TYPE_NONE）
    void set_cur_control_stat(uint32_t stat);
    uint32_t get_cur_control_stat() const;
    /// 按 MoveTypeMap 校验后请求切换；成功返回 0，拒绝返回 -1
    int request_mot_type(RobotMotType target);
    /**********************************************************************/

    /***************控制源切换（按键 / loco 互斥）**************/
    void set_control_source(ControlSource src);
    ControlSource get_control_source() const;
    /******************************************************/

    /***************RobotJointGroups 记录（默认关闭）**************/
    /// 启用/关闭记录（线程安全）
    void enable_record_robot_groups(bool enable);
    /// 设置记录文件路径（为空则使用默认 /tmp/robot_joint_groups.csv）
    void set_record_robot_groups_path(std::string path);
    /******************************************************/

    /***************Record (robotgroups + imu) shared activation**************/
    /// 统一激活接口：同时控制 robotgroups 与 imu 记录开关
    void enable_record(bool enable);
    /// 设置 IMU 记录文件路径（为空则使用默认 /tmp/imu.csv）
    void set_record_imu_path(std::string path);
    /***********************************************************************/

    /***************按键扫描接口（可选）**************/
    /// 启动按键扫描（若 Options.enable_key_scan=true 且 auto_start_spin=true，会在构造时自动启动）
    int start_key_scan();
    /// 停止按键扫描
    int stop_key_scan();
    /// 直接设置按键->motion 的映射表（线程安全）
    void set_key_motion_map(std::unordered_map<char, RobotMotType> map);
    /// 从配置文件加载映射表（预留：当前实现会清空并保持为空，返回 0）
    int load_key_motion_map_from_config(const std::string& path);
    /**********************************************/

    /***************RobotMotType Service & Callback**************/
    using MotTypeCallback = std::function<void(RobotMotType)>;
    /// 注册回调：当收到 service 请求设置 mot_type 时会调用（线程安全）
    void set_mot_type_callback(MotTypeCallback cb);
    /************************************************************/

    /***************GoHome Service & Callback**************/
    using GoHomeCallback = std::function<void()>;
    /// 注册回调：当收到 go_home service 请求时会调用（线程安全）
    void set_go_home_callback(GoHomeCallback cb);
    /******************************************************/

    /***************loco_move topic & Callback**************/
    using LocoMoveCallback = std::function<void(const LocoMoveCmd&)>;
    /// 注册回调：当收到 loco_move topic 时会调用（线程安全）
    void set_loco_move_callback(LocoMoveCallback cb);
    /// 获取最新 loco_move 指令；若尚未收到过则返回 false
    bool get_latest_loco_move(LocoMoveCmd& out) const;
    /******************************************************/

    /***************外部头/爪 JointTrajectory 指令缓存**************/
    void enable_ext_joint_command(bool enable);
    bool is_ext_joint_command_active() const;
    /// KeepLatest；未激活或无有效帧时返回 false 且不修改 out
    bool get_ext_joint_command(trajectory_msgs::msg::JointTrajectory& out) const;
    /**************************************************************/

    /***************BasicConfigs getters**************/
    BasicConfigs get_basic_configs() const;
    std::vector<double> get_home_positions() const;
    double get_max_walking_speed() const;
    /************************************************/

    /***************MoveTypeMap（模式A 状态机切换表）**************/
    /// 内置默认状态机切换表（与 configs/move_type_map.ini 一致）
    static MoveTypeMap default_move_type_map();
    /// 获取当前切换表（线程安全）
    MoveTypeMap get_move_type_map() const;
    /// 设置切换表并持久化到 ini（无 ROS service，仅 C++）；返回 0 成功，-1 写文件失败
    int set_move_type_map(MoveTypeMap map);
    /// 从 ini 重新加载；文件无效时回退默认表
    int load_move_type_map_from_config(const std::string& path = "");
    /***************************************************************/

    /***************离线语音播放（voice_player）**********************/
    /// 在指定目录下异步入队播放相对路径 wav（如 "mot/walk.wav"）。
    /// 返回 0：已入队；-1：参数非法或未就绪。调用方不阻塞；文件校验与 /voice_control 在 worker 线程执行。
    int play_voice_clip(const std::string& clips_dir, const std::string& relative_clip);
    int play_voice_clips(const std::string& clips_dir, const std::vector<std::string>& relative_clips);
    /// 停止当前播放并清空待播队列（IntfVoice 本地队列 + voice_player）。调用方仅入队，不阻塞。
    int stop_voice_playback();
    /***************************************************************/

private:
    std::unique_ptr<RobotImpl> impl_;
};

}  // namespace legged_waist_controller