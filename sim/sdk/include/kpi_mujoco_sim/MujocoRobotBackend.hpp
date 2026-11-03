#pragma once

#include "kpi_mujoco_sim/robot_intf.hpp"

#include <mujoco/mujoco.h>

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class MujocoRobotBackend final : public legged_waist_controller::RobotIntf::Backend
{
public:
    struct Config
    {
        std::string xml_path;
        std::vector<std::string> joint_names;
        std::vector<double> init_target;
        std::vector<double> kp;
        std::vector<double> kd;
        std::vector<double> kp_pitch_roll;
        std::vector<double> kd_pitch_roll;
        int angle_map{0};
        double simulation_dt{0.001};
        int control_decimation{20};
        std::vector<double> init_base_pos{0.0, 0.0, 0.60};
        std::vector<double> init_base_quat{1.0, 0.0, 0.0, 0.0};
        std::string root_body_name{"base_link"};
        std::string imu_body_name{"waist_yaw_Link"};
        bool use_imu_body{false};
        std::string log_path;
    };

    explicit MujocoRobotBackend(Config config);
    ~MujocoRobotBackend() override;

    MujocoRobotBackend(const MujocoRobotBackend &) = delete;
    MujocoRobotBackend &operator=(const MujocoRobotBackend &) = delete;

    int start() override;
    int stop() override;
    int do_enable(bool enable) override;
    int enable_pd(bool enable) override;
    int set_joint_operation_mode(std::int8_t mode) override;
    int set_joint_mit_gains(const std::string &joint_name, double kp, double kd) override;
    int set_joint_pd_para(const std::string &joint_name, double p, double d) override;
    int set_joint_angle_limit(const std::string &joint_name, double min_angle, double max_angle) override;
    int get_joint_status(const std::string &joint_name, legged_waist_controller::RobotIntf::JointStatus &status) const override;
    bool has_imu() const override;
    bool get_imu(legged_waist_controller::RobotIntf::ImuData &out) const override;
    bool get_joy_robot_state(legged_waist_controller::RobotIntf::JoyRobotState &out) const override;
    int set_joint_trajectory(const std::vector<std::string> &joint_names,
                             const trajectory_msgs::msg::JointTrajectoryPoint &traj) override;

    void set_joy_state(const legged_waist_controller::RobotIntf::JoyRobotState &joy);
    void set_fsm_state(int state);
    void step_physics();
    void sync_wall_time(double step_start_seconds) const;
    double simulation_time() const;
    int control_decimation() const { return config_.control_decimation; }
    double simulation_dt() const { return config_.simulation_dt; }
    mjModel *model() const { return model_; }
    mjData *data() const { return data_; }

private:
    static std::string deploy_to_mujoco_joint_name(const std::string &deploy_name);
    static void pitch_roll_to_drive(double pitch, double roll, double &a, double &b);
    static void convert_drive_to_pitch_roll(std::vector<double> &q, std::vector<double> &dq,
                                            double left_j[2][2], double right_j[2][2]);
    static void apply_ankle_tau_map(std::vector<double> &tau, const double left_j[2][2], const double right_j[2][2]);
    static void compensate_for_controller_imu_process(const double desired_quat[4], const double desired_omega[3],
                                                      legged_waist_controller::RobotIntf::ImuData &out);

    void initialize_model();
    void initialize_log();
    void disable_base_weld_if_present() const;
    void read_raw_joint_state(std::vector<double> &q, std::vector<double> &dq) const;
    void write_log_row(const std::vector<double> &q_raw, const std::vector<double> &dq_raw,
                       const std::vector<double> &tau);

    Config config_;
    mjModel *model_{nullptr};
    mjData *data_{nullptr};
    std::vector<int> qpos_addr_;
    std::vector<int> dof_addr_;
    std::unordered_map<std::string, int> joint_index_by_name_;
    std::vector<double> target_;
    std::vector<double> pos_cmd_;
    std::vector<double> kp_;
    std::vector<double> kd_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
    bool enabled_{false};
    bool pd_enabled_{false};
    std::int8_t operation_mode_{legged_waist_controller::RobotIntf::JOINT_OPERATION_MODE_MIT};
    mutable std::mutex mutex_;
    legged_waist_controller::RobotIntf::JoyRobotState joy_;
    int fsm_state_{0};
    std::ofstream log_;
    std::uint64_t sim_step_{0};
};
