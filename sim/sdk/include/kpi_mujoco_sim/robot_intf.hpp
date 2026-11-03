#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace legged_waist_controller
{

    enum class RobotMotType
    {
        MOT_TYPE_NONE = 0,
        MOT_TYPE_ENABLE = 901,
        MOT_TYPE_DISABLE = 902,
        MOT_TYPE_RESET = 903,
        MOT_TYPE_GO_HOME = 904,
        MOT_TYPE_GO_LOCO_INITPOS = 905,
        MOT_TYPE_GO_MIMIC_INITPOS = 906,
        MOT_TYPE_PLAY_MIMIC = 907,
        MOT_TYPE_ABORT = 999,
        MOT_TYPE_SWITCH_TO_FALUT = 1,
        MOT_TYPE_SWITCH_TO_START_FROM_LAYUP = 2,
        MOT_TYPE_SWITCH_TO_START_FROM_LAYDOWN = 3,
        MOT_TYPE_SWITCH_TO_IDLE = 7,
        MOT_TYPE_SWITCH_TO_LOCO = 8,
        MOT_TYPE_SWITCH_TO_MIMIC = 9,
        MOT_TYPE_SWITCH_TO_NOTARGET_HOLD = 10,
        MOT_TYPE_SWITCH_TO_NOTARGET = 11,
        MOT_TYPE_SWITCH_TO_REMOTE = 12,
        MOT_TYPE_SWITCH_TO_USER = 13,
        MOT_TYPE_SWITCH_TO_FSM_JAKAMINIMIMIC = 14,
        MOT_TYPE_NOD_HEAD = 101,
        MOT_TYPE_SHAKE_HEAD = 102,
        MOT_TYPE_WAVEHAND_LEFT = 103,
        MOT_TYPE_WAVEHAND_RIGHT = 104,
        MOT_TYPE_WAVEHANDS_LOW = 105,
        MOT_TYPE_WAVEHANDS_HIGH = 106,
        MOT_TYPE_SOLUTE_RIGHT = 107,
        MOT_TYPE_SHAKEHAND_RIGHT = 108,
        MOT_TYPE_HUG = 109,
        MOT_TYPE_YAOTOUHUANGNAO = 110,
        MOT_TYPE_CLAPHANDS = 111,
        MOT_TYPE_SHY = 112,
        MOT_TYPE_GUANWANG = 113,
        MOT_TYPE_CHEERS = 114,
        MOT_TYPE_HUAXIN = 117,
        MOT_TYPE_JIAORENGUOLAI = 118,
        MOT_TYPE_JIAYOU = 119,
        MOT_TYPE_JUJUE = 120,
        MOT_TYPE_JUSHOU = 121,
        MOT_TYPE_KEAI = 122,
        MOT_TYPE_NAOTOU = 123,
        MOT_TYPE_QIDAI = 124,
        MOT_TYPE_SIKAO = 125,
        MOT_TYPE_WULIAO = 126,
    };

    struct MotCommand
    {
        RobotMotType mot_type{RobotMotType::MOT_TYPE_NONE};
        uint64_t seq_id{0};
    };

    class RobotIntf
    {
    public:
        static constexpr std::int8_t JOINT_OPERATION_MODE_CSP = 8;
        static constexpr std::int8_t JOINT_OPERATION_MODE_MIT = 12;

        enum class ControlSource : uint8_t
        {
            KEY = 0,
            JOY = 1,
            APP = 2,
        };

        struct OperationCodeSet
        {
            std::string name;
            uint32_t code{0};
        };
        using MoveTypeMap = std::unordered_map<uint32_t, std::vector<OperationCodeSet>>;

        struct JointStatus
        {
            std::string name;
            uint8_t slave_id{0};
            double pos_cmd{0.0};
            double pos_fdb{0.0};
            double vel_fdb{0.0};
            double effort_fdb{0.0};
            bool is_enable{false};
            bool is_fault{false};
            uint64_t error_code{0};
            bool ethercat_operational{false};
        };

        struct JoyStickState
        {
            float axes_L_X{0.0f};
            float axes_L_Y{0.0f};
            float axes_R_X{0.0f};
            float axes_R_Y{0.0f};
            float axes_LT{0.0f};
            float axes_RT{0.0f};
            float axes_DX{0.0f};
            float axes_DY{0.0f};
            bool buttons_A{false};
            bool buttons_B{false};
            bool buttons_X{false};
            bool buttons_Y{false};
            bool buttons_LB{false};
            bool buttons_RB{false};
            bool buttons_BACK{false};
            bool buttons_START{false};
        };

        struct JoyRobotState
        {
            bool joy_online{true};
            JoyStickState joy;
        };

        struct ImuData
        {
            int32_t stamp_sec{0};
            uint32_t stamp_nanosec{0};
            std::string frame_id;
            double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
            double wx{0.0}, wy{0.0}, wz{0.0};
            double ax{0.0}, ay{0.0}, az{0.0};
        };

        struct LocoMoveCmd
        {
            double forward{0.0};
            double lateral{0.0};
            double yaw{0.0};
        };

        struct Options
        {
            bool enable_joy_sub{false};
            bool auto_start_spin{true};
            std::chrono::milliseconds joy_timeout{500};
        };

        class Backend
        {
        public:
            virtual ~Backend() = default;
            virtual int start() = 0;
            virtual int stop() = 0;
            virtual int do_enable(bool enable) = 0;
            virtual int enable_pd(bool enable) = 0;
            virtual int set_joint_operation_mode(std::int8_t mode) = 0;
            virtual int set_joint_mit_gains(const std::string &joint_name, double kp, double kd) = 0;
            virtual int set_joint_pd_para(const std::string &joint_name, double p, double d) = 0;
            virtual int set_joint_angle_limit(const std::string &joint_name, double min_angle, double max_angle) = 0;
            virtual int get_joint_status(const std::string &joint_name, JointStatus &status) const = 0;
            virtual bool has_imu() const = 0;
            virtual bool get_imu(ImuData &out) const = 0;
            virtual bool get_joy_robot_state(JoyRobotState &out) const = 0;
            virtual int set_joint_trajectory(const std::vector<std::string> &joint_names,
                                             const trajectory_msgs::msg::JointTrajectoryPoint &traj) = 0;
        };

        RobotIntf();
        explicit RobotIntf(const Options &opt);
        ~RobotIntf();

        RobotIntf(const RobotIntf &) = delete;
        RobotIntf &operator=(const RobotIntf &) = delete;
        RobotIntf(RobotIntf &&) noexcept;
        RobotIntf &operator=(RobotIntf &&) noexcept;

        void attach_backend(std::shared_ptr<Backend> backend);

        int start();
        int stop();
        int do_enable(bool enable);
        int enable_pd(bool enable);
        int set_joint_operation_mode(std::int8_t mode);
        int set_joint_mit_gains(const std::string &joint_name, double kp, double kd);
        int set_joint_pd_para(const std::string &joint_name, double p, double d);
        int set_joint_angle_limit(const std::string &joint_name, double min_angle, double max_angle);
        int get_joint_status(const std::string &joint_name, JointStatus &status) const;
        int clear_error();
        bool has_imu() const;
        bool get_imu(ImuData &out) const;
        bool get_joy_robot_state(JoyRobotState &out) const;
        ControlSource get_control_source() const;
        void set_control_source(ControlSource source);
        bool poll_mot_command(MotCommand &out);
        void set_cur_control_stat(uint32_t stat);
        bool get_latest_loco_move(LocoMoveCmd &out) const;
        std::vector<double> get_home_positions() const;
        int set_move_type_map(MoveTypeMap map);
        int set_joint_trajectory(const trajectory_msgs::msg::JointTrajectoryPoint &traj);
        int set_joint_trajectory(const std::vector<std::string> &joint_names,
                                 const trajectory_msgs::msg::JointTrajectoryPoint &traj);

    private:
        Options options_;
        std::shared_ptr<Backend> backend_;
        ControlSource control_source_{ControlSource::JOY};
        uint32_t cur_control_stat_{0};
        MoveTypeMap move_type_map_;
    };

} // namespace legged_waist_controller
