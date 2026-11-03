// Khan SuperMimic packed actor deployment. Deliberately follows the existing
// FSMMimicJakaMiniZmq control/observation/history structure. Real hardware
// joint parameters and observation/action scales come from KpiConfig.yaml.
#include "RLController.hpp"
#include "StatePublisher.hpp"
#include "SuperRealtimeMotionBuffer.hpp"
#include "SuperMimicSafety.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

namespace RLController
{

    extern fsm_func_ptr StateJudge;
    extern fsm_func_ptr StateRun;
    extern FSMSTATE ReqState;

    namespace
    {
        constexpr int kJakaMiniSuperZMQCommandDim = 155;
        constexpr int kJakaMiniSuperZMQAnchorOriDim = 30;
        constexpr int kJakaMiniSuperZMQGravityDim = 3;
        constexpr int kJakaMiniSuperZMQAngVelDim = 3;
        constexpr int kJakaMiniSuperZMQHistObsDim = 87;
        constexpr int kJakaMiniSuperZMQFutureSteps = 5;
        constexpr int kJakaMiniSuperZMQRot6dStride = 6;
        constexpr int kJakaMiniSuperZMQDefaultFrameStack = 5;
        constexpr float kJakaMiniSuperZMQPi = 3.14159265358979323846f;


        // Timing configuration
        constexpr int kDefaultTimingPrintInterval = 25;   // Print every 25 steps (0.5s at 50Hz)
        constexpr double kLatencySpikeThresholdMs = 15.0; // Spike warning threshold (ms)

        // 部署序(joint_names 配置序) → JAKA 序(SIMPLE 端 JAKA_JOINT_NAMES 序)的索引映射。
        // JAKA 序: 左腿6 | 右腿6 | 腰1 | 左臂6 | 右臂6 | 颈2
        // 部署序: 左腿6 | 左臂6 | 右腿6 | 右臂6 | 腰1 | 颈2
        constexpr std::array<int, 27> kDeployToJakaIndex = {
            0, 1, 2, 3, 4, 5,          // 左腿
            12, 13, 14, 15, 16, 17,   // 右腿
            24,                       // waist_yaw
            6, 7, 8, 9, 10, 11,       // 左臂
            18, 19, 20, 21, 22, 23,   // 右臂
            25, 26                    // neck_yaw / neck_pitch
        };

    } // anonymous namespace

    static legged_waist_controller::RobotIntf::JoyRobotState zmq_joydata;
    static legged_waist_controller::RobotIntf::ImuData zmq_imu;
    static double zmq_q_cmd[50], zmq_q[50], zmq_qd[50];
    static double zmq_omega[3], zmq_acc[3], zmq_quat[4];
    static bool zmq_exit_chord_released;
    static bool zmq_first_step;
    static uint64_t zmq_last_generation;
    static MotionWindow::Status zmq_last_buffer_status;
    static int zmq_frame_stack;
    static Eigen::Quaternionf zmq_align_quat(1.0f, 0.0f, 0.0f, 0.0f);
    static std::vector<float> zmq_single_obs;
    static std::vector<float> zmq_stacked_obs;
    static std::vector<float> zmq_last_action;
    static std::deque<std::vector<float>> zmq_obs_history;

    static uint64_t zmq_timing_step_count = 0;
    static double zmq_timing_obs_sum = 0.0;
    static double zmq_timing_infer_sum = 0.0;
    static double zmq_timing_action_sum = 0.0;
    static double zmq_timing_total_sum = 0.0;
    static double zmq_timing_obs_max = 0.0;
    static double zmq_timing_infer_max = 0.0;
    static double zmq_timing_action_max = 0.0;
    static double zmq_timing_total_max = 0.0;

    static std::unique_ptr<SuperRealtimeMotionBuffer> zmq_motion_buffer;

    // OpenHLM 风格 state 发布器(50Hz tick 内联、非阻塞; 部署序→JAKA 序重排)
    static std::unique_ptr<StatePublisher> zmq_state_pub;
    // Opt-in numerical parity tape; disabled in normal deployment.
    static std::ofstream zmq_policy_diagnostic;

    static void from_fsm_mimic_jaka_mini_superzmq();

    static inline Eigen::Quaternionf zmq_normalize_quat(const Eigen::Quaternionf &q)
    {
        if (q.norm() < 1.0e-6f)
            return Eigen::Quaternionf::Identity();
        Eigen::Quaternionf r = q;
        r.normalize();
        return r;
    }

    static inline Eigen::Vector3f zmq_quat_rotate_inverse(const Eigen::Quaternionf &q, const Eigen::Vector3f &v)
    {
        return zmq_normalize_quat(q).conjugate() * v;
    }

    static inline std::array<float, kJakaMiniSuperZMQRot6dStride> zmq_quat_to_rot6d(const Eigen::Quaternionf &q)
    {
        const Eigen::Matrix3f rot = zmq_normalize_quat(q).toRotationMatrix();
        return {rot(0, 0), rot(0, 1), rot(1, 0), rot(1, 1), rot(2, 0), rot(2, 1)};
    }

    static inline Eigen::Quaternionf zmq_robot_quat_from_imu()
    {
        return zmq_normalize_quat(Eigen::Quaternionf(
            static_cast<float>(zmq_quat[0]), static_cast<float>(zmq_quat[1]),
            static_cast<float>(zmq_quat[2]), static_cast<float>(zmq_quat[3])));
    }

    static inline bool zmq_resolve_policy_layout()
    {
        if (mMimicJakaMiniSuperZmqPolicy.session == nullptr || !mJakaMiniSuperDeployment)
        {
            std::cerr << "[FSMJakaMiniSuperZmq] policy session is null." << std::endl;
            return false;
        }

        zmq_frame_stack = kJakaMiniSuperZMQDefaultFrameStack;
        if (mMimicJakaMiniSuperZmqPolicy.input_shape[1] > 0)
        {
            const int non_history_dim = kJakaMiniSuperZMQCommandDim + kJakaMiniSuperZMQAnchorOriDim;
            const int remaining = static_cast<int>(mMimicJakaMiniSuperZmqPolicy.input_shape[1]) - non_history_dim;
            if (remaining <= 0 || remaining % kJakaMiniSuperZMQHistObsDim != 0)
            {
                std::cerr << "[FSMJakaMiniSuperZmq] input dim mismatch" << std::endl;
                return false;
            }
            zmq_frame_stack = remaining / kJakaMiniSuperZMQHistObsDim;
        }

        if (zmq_frame_stack != 5 || mNumAction != 27 ||
            mMimicJakaMiniSuperZmqPolicy.input_shape[1] != 620 ||
            static_cast<int>(mMimicJakaMiniSuperZmqPolicy.output_shape[1]) != mNumAction)
        {
            std::cerr << "[FSMJakaMiniSuperZmq] output dim mismatch" << std::endl;
            return false;
        }

        zmq_single_obs.assign(kJakaMiniSuperZMQHistObsDim, 0.0f);
        zmq_stacked_obs.assign(static_cast<int>(mMimicJakaMiniSuperZmqPolicy.input_shape[1]), 0.0f);
        zmq_last_action.assign(mNumAction, 0.0f);
        zmq_obs_history.clear();

        std::cerr << "[FSMJakaMiniSuperZmq] action_dim=" << mNumAction
                  << ", frame_stack=" << zmq_frame_stack << std::endl;
        return true;
    }

    static inline bool zmq_fetch_robot_state()
    {
        if (!mRobot || !mDataReadyFlag)
        {
            latch_super_fault("SuperMimic feedback", "robot feedback is not ready");
            return false;
        }
        int enabled = 0, faulted = 0;
        for (int i = 0; i < mNumAction; ++i)
        {
            legged_waist_controller::RobotIntf::JointStatus state{};
            if (mRobot->get_joint_status(mJointNames[i], state) < 0 ||
                !std::isfinite(state.pos_cmd) || !std::isfinite(state.pos_fdb) ||
                !std::isfinite(state.vel_fdb))
            {
                latch_super_fault("SuperMimic feedback", ("missing/nonfinite joint: " + mJointNames[i]).c_str());
                return false;
            }
            enabled += state.is_enable ? 1 : 0;
            faulted += state.is_fault ? 1 : 0;
            zmq_q_cmd[i] = state.pos_cmd;
            zmq_q[i] = state.pos_fdb;
            zmq_qd[i] = state.vel_fdb;
        }
        mEnabledJointCount = enabled;
        mFaultJointCount = faulted;
        if (enabled != mNumAction || faulted > 0)
        {
            latch_super_fault("SuperMimic feedback", "joint disabled or faulted");
            return false;
        }

        if (!decode_super_parallel_ankles(zmq_q, zmq_qd, "SuperMimic ZMQ feedback"))
            return false;

        if (mRobot->get_imu(zmq_imu))
        {
            const std::array<double, 10> raw_imu = {
                zmq_imu.qw, zmq_imu.qx, zmq_imu.qy, zmq_imu.qz,
                zmq_imu.wx, zmq_imu.wy, zmq_imu.wz,
                zmq_imu.ax, zmq_imu.ay, zmq_imu.az};
            const double quat_norm2 = zmq_imu.qw * zmq_imu.qw + zmq_imu.qx * zmq_imu.qx +
                                      zmq_imu.qy * zmq_imu.qy + zmq_imu.qz * zmq_imu.qz;
            if (!std::all_of(raw_imu.begin(), raw_imu.end(), [](double value) { return std::isfinite(value); }) ||
                !std::isfinite(quat_norm2) || quat_norm2 < 1.e-12)
            {
                latch_super_fault("SuperMimic IMU", "nonfinite data or invalid quaternion");
                return false;
            }
            zmq_acc[0] = zmq_imu.ax;
            zmq_acc[1] = zmq_imu.ay;
            zmq_acc[2] = zmq_imu.az;
            zmq_omega[0] = zmq_imu.wx;
            zmq_omega[1] = zmq_imu.wy;
            zmq_omega[2] = zmq_imu.wz;
            zmq_quat[0] = zmq_imu.qw;
            zmq_quat[1] = zmq_imu.qx;
            zmq_quat[2] = zmq_imu.qy;
            zmq_quat[3] = zmq_imu.qz;
            imu_process(zmq_acc, zmq_omega, zmq_quat);

            // Match the installed base-link IMU axes using the same real-robot
            // calibration as the existing FSM. SuperMimic has a base_link anchor;
            // do not append a waist theta/dtheta transform.
        }
        else
        {
            latch_super_fault("SuperMimic IMU", "no IMU sample available");
            return false;
        }
        return true;
    }

    static inline void zmq_initialize_alignment(const InterpolatedFrame &frame)
    {
        const Eigen::Quaternionf rob_yaw0 = yawQuaternion(zmq_robot_quat_from_imu());
        const Eigen::Quaternionf ref_yaw0 = yawQuaternion(zmq_normalize_quat(frame.root_quat_w));
        zmq_align_quat = zmq_normalize_quat(rob_yaw0 * ref_yaw0.conjugate());
    }

    static inline void zmq_initialize_last_action()
    {
        // Training reset contract: previous RAW policy action is zero.
        std::fill(zmq_last_action.begin(), zmq_last_action.end(), 0.0f);
    }

    static inline std::array<float, kJakaMiniSuperZMQCommandDim> zmq_construct_command(
        const std::array<InterpolatedFrame, SuperRealtimeMotionBuffer::kFutureSteps> &frames)
    {
        std::array<float, kJakaMiniSuperZMQCommandDim> cmd{};
        const Eigen::Vector3f anchor_pos = frames[0].root_pos_w;
        const Eigen::Quaternionf anchor_quat = zmq_normalize_quat(frames[0].root_quat_w);

        int offset = 0;
        for (int i = 0; i < kJakaMiniSuperZMQFutureSteps; ++i)
        {
            const Eigen::Vector3f diff_w = frames[i].root_pos_w - anchor_pos;
            const Eigen::Vector3f diff_b = zmq_quat_rotate_inverse(anchor_quat, diff_w);
            cmd[offset++] = diff_b[0];
            cmd[offset++] = diff_b[1];
            cmd[offset++] = diff_b[2];
        }

        for (int i = 0; i < kJakaMiniSuperZMQFutureSteps; ++i)
            cmd[offset++] = frames[i].root_pos_w[2];

        for (int i = 0; i < kJakaMiniSuperZMQFutureSteps; ++i)
        {
            for (int policy_idx = 0; policy_idx < mNumAction; ++policy_idx)
            {
                float jv = 0.0f;
                const int buffer_idx = mJakaMiniSuperDeployment->policy_from_buffer[policy_idx];
                if (buffer_idx < frames[i].dof_pos.size())
                    jv = frames[i].dof_pos[buffer_idx];
                cmd[offset++] = jv;
            }
        }

        return cmd;
    }

    static inline std::array<float, kJakaMiniSuperZMQAnchorOriDim> zmq_construct_anchor_orientation(
        const std::array<InterpolatedFrame, SuperRealtimeMotionBuffer::kFutureSteps> &frames)
    {
        const Eigen::Quaternionf robot_quat_now = zmq_robot_quat_from_imu();
        std::array<float, kJakaMiniSuperZMQAnchorOriDim> ori{};

        int offset = 0;
        for (int i = 0; i < kJakaMiniSuperZMQFutureSteps; ++i)
        {
            const Eigen::Quaternionf ref_quat = zmq_normalize_quat(frames[i].root_quat_w);
            const Eigen::Quaternionf anchor_world = zmq_normalize_quat(zmq_align_quat * ref_quat);
            const Eigen::Quaternionf ori_body = zmq_normalize_quat(robot_quat_now.conjugate() * anchor_world);
            const auto rot6d = zmq_quat_to_rot6d(ori_body);
            for (int j = 0; j < kJakaMiniSuperZMQRot6dStride; ++j)
                ori[offset++] = rot6d[j];
        }
        return ori;
    }

    static inline std::array<float, kJakaMiniSuperZMQGravityDim> zmq_construct_gravity()
    {
        return {
            static_cast<float>(2.0 * (-zmq_quat[3] * zmq_quat[1] + zmq_quat[0] * zmq_quat[2])),
            static_cast<float>(-2.0 * (zmq_quat[3] * zmq_quat[2] + zmq_quat[0] * zmq_quat[1])),
            static_cast<float>(1.0 - 2.0 * (zmq_quat[0] * zmq_quat[0] + zmq_quat[3] * zmq_quat[3])),
        };
    }

    static inline void zmq_construct_single_obs()
    {
        const auto gravity = zmq_construct_gravity();
        std::fill(zmq_single_obs.begin(), zmq_single_obs.end(), 0.0f);

        int off = 0;
        std::copy(gravity.begin(), gravity.end(), zmq_single_obs.begin() + off);
        off += kJakaMiniSuperZMQGravityDim;

        for (int i = 0; i < kJakaMiniSuperZMQAngVelDim; ++i)
            zmq_single_obs[off + i] = static_cast<float>(zmq_omega[i] * mJakaMiniSuperDeployment->angular_velocity_scale);
        off += kJakaMiniSuperZMQAngVelDim;

        for (int policy_idx = 0; policy_idx < mNumAction; ++policy_idx)
        {
            const int deploy_idx = mJakaMiniSuperDeployment->policy_to_deploy[policy_idx];
            zmq_single_obs[off + policy_idx] = static_cast<float>(zmq_q[deploy_idx] - mJakaMiniSuperDeployment->default_joint_pos[policy_idx]);
        }
        off += mNumAction;

        for (int policy_idx = 0; policy_idx < mNumAction; ++policy_idx)
        {
            const int deploy_idx = mJakaMiniSuperDeployment->policy_to_deploy[policy_idx];
            zmq_single_obs[off + policy_idx] = static_cast<float>(zmq_qd[deploy_idx] * mJakaMiniSuperDeployment->joint_velocity_scale);
        }
        off += mNumAction;

        for (int i = 0; i < mNumAction; ++i)
            zmq_single_obs[off + i] = zmq_last_action[i];
    }

    static inline void zmq_init_history()
    {
        zmq_obs_history.clear();
        for (int i = 0; i < zmq_frame_stack; ++i)
            zmq_obs_history.push_back(zmq_single_obs);
    }

    static inline void zmq_push_history()
    {
        if (static_cast<int>(zmq_obs_history.size()) == zmq_frame_stack)
            zmq_obs_history.pop_front();
        zmq_obs_history.push_back(zmq_single_obs);
    }

    static inline void zmq_pack_history(
        const std::array<InterpolatedFrame, SuperRealtimeMotionBuffer::kFutureSteps> &frames)
    {
        const auto command = zmq_construct_command(frames);
        const auto anchor_ori = zmq_construct_anchor_orientation(frames);

        static constexpr std::array<int, 5> hist_blocks = {3, 3, 27, 27, 27};

        std::fill(zmq_stacked_obs.begin(), zmq_stacked_obs.end(), 0.0f);

        int w = 0;
        std::copy(command.begin(), command.end(), zmq_stacked_obs.begin() + w);
        w += kJakaMiniSuperZMQCommandDim;
        std::copy(anchor_ori.begin(), anchor_ori.end(), zmq_stacked_obs.begin() + w);
        w += kJakaMiniSuperZMQAnchorOriDim;

        int r = 0;
        for (int bs : hist_blocks)
        {
            for (const auto &frame : zmq_obs_history)
            {
                std::copy_n(frame.begin() + r, bs, zmq_stacked_obs.begin() + w);
                w += bs;
            }
            r += bs;
        }

        std::copy(zmq_stacked_obs.begin(), zmq_stacked_obs.end(),
                  mMimicJakaMiniSuperZmqPolicy.input_buffer);
    }

    static inline bool zmq_fill_trajectory()
    {
        // Previous actions retain the raw network values. Physical targets use
        // the existing real YAML position limits, before optional PR -> AB.
        for (int policy_idx = 0; policy_idx < mNumAction; ++policy_idx)
            if (!std::isfinite(mMimicJakaMiniSuperZmqPolicy.output_buffer[policy_idx]))
                return false;
        for (int policy_idx = 0; policy_idx < mNumAction; ++policy_idx)
        {
            const int deploy_idx = mJakaMiniSuperDeployment->policy_to_deploy[policy_idx];
            zmq_last_action[policy_idx] = mMimicJakaMiniSuperZmqPolicy.output_buffer[policy_idx];
            mPt.positions[deploy_idx] =
                static_cast<double>(zmq_last_action[policy_idx]) *
                    mJakaMiniSuperDeployment->action_scale[policy_idx] +
                    mJakaMiniSuperDeployment->default_joint_pos[policy_idx];
        }
        if (!std::all_of(mPt.positions.begin(), mPt.positions.end(),
                         [](double value) { return std::isfinite(value); }))
            return false;
        bool limited = false;
        for (int deploy = 0; deploy < mNumAction; ++deploy)
        {
            const double raw_target = mPt.positions[deploy];
            mPt.positions[deploy] = std::clamp(raw_target, mDownLmt[deploy], mUpLmt[deploy]);
            limited = limited || raw_target != mPt.positions[deploy];
        }
        if (limited && zmq_timing_step_count % 50 == 0)
            std::cerr << "[FSMJakaMiniSuperZmq] physical targets limited by real YAML joint bounds" << std::endl;

        if (mAngleMapFlag && !encode_super_parallel_ankle_targets(mPt.positions.data(), "SuperMimic ZMQ target"))
            return false;
        mPt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(mTimeStep));
        return std::all_of(mPt.positions.begin(), mPt.positions.end(),
                           [](double value) { return std::isfinite(value); });
    }

    static void zmq_write_policy_diagnostic(const MotionWindow &window, bool reset_history)
    {
        if (!zmq_policy_diagnostic.is_open()) return;
        nlohmann::json frames = nlohmann::json::array();
        for (const auto &frame : window.frames)
            frames.push_back({{"root_pos", {frame.root_pos_w.x(), frame.root_pos_w.y(), frame.root_pos_w.z()}},
                              {"root_quat", {frame.root_quat_w.w(), frame.root_quat_w.x(), frame.root_quat_w.y(), frame.root_quat_w.z()}},
                              {"joint_pos_buffer", std::vector<float>(frame.dof_pos.data(), frame.dof_pos.data() + frame.dof_pos.size())}});
        const nlohmann::json record = {
            {"step", zmq_timing_step_count}, {"query_steady_ns", window.query_steady_ns},
            {"generation", window.generation}, {"status", SuperRealtimeMotionBuffer::status_name(window.status)},
            {"reset_history", reset_history}, {"frames", frames},
            {"robot_quat", {zmq_quat[0], zmq_quat[1], zmq_quat[2], zmq_quat[3]}},
            {"omega", {zmq_omega[0], zmq_omega[1], zmq_omega[2]}},
            {"q_deploy", std::vector<double>(zmq_q, zmq_q + mNumAction)},
            {"qd_deploy", std::vector<double>(zmq_qd, zmq_qd + mNumAction)},
            {"align_quat", {zmq_align_quat.w(), zmq_align_quat.x(), zmq_align_quat.y(), zmq_align_quat.z()}},
            {"obs", zmq_stacked_obs}, {"actions", zmq_last_action}, {"targets_deploy", mPt.positions}};
        zmq_policy_diagnostic << record.dump() << '\n';
    }

    static void fsm_mimic_jaka_mini_superzmq_judge()
    {
        const bool joystick_owned = mRobot->get_control_source() ==
            legged_waist_controller::RobotIntf::ControlSource::JOY;
        const bool joystick_ready = mRobot->get_joy_robot_state(zmq_joydata) && zmq_joydata.joy_online;
        if (!joystick_owned || !joystick_ready)
            latch_super_fault("SuperMimic joystick", "JOY ownership lost or joystick watchdog expired");
        if (mErrCode != ERR_NONE)
        {
            ReqState = FSM_FAULT;
        }
        else
        {
            if (zmq_joydata.joy.buttons_A)
            {
                mRobot->do_enable(false);
                ReqState = FSM_FAULT;
            }

            // Entry also uses LB+B. Require release before this chord can exit,
            // so the held entry buttons cannot immediately return to IDLE.
            const bool exit_chord = zmq_joydata.joy.buttons_LB && zmq_joydata.joy.buttons_B;
            if (!exit_chord) zmq_exit_chord_released = true;
            if (ReqState == FSM_JAKAMINIMIMIC_SUPERZMQ && zmq_exit_chord_released && exit_chord)
                ReqState = FSM_IDLE;
        }

        if (ReqState != FSM_JAKAMINIMIMIC_SUPERZMQ)
        {
            from_fsm_mimic_jaka_mini_superzmq();
        }
    }

    static void fsm_mimic_jaka_mini_superzmq_run()
    {
        const auto t_step_start = std::chrono::steady_clock::now();

        if (!zmq_fetch_robot_state())
        {
            mErrCode = ERR_JOINT;
            return;
        }

        // OpenHLM 风格: 每个控制 tick 内联、非阻塞发布 state(部署序→JAKA 序),
        // 供独立录制进程(record_jaka_zmq)订阅。zmq_quat 为 imu_process 后的
        // base 姿态四元数(wxyz); 踝关节 zmq_q[4,5,16,17] 已是 pitch/roll 语义。
        if (zmq_state_pub)
        {
            std::array<double, 27> jaka_q{};
            for (int i = 0; i < 27; ++i)
                jaka_q[i] = zmq_q[kDeployToJakaIndex[i]];
            zmq_state_pub->publish(
                jaka_q,
                {zmq_quat[0], zmq_quat[1], zmq_quat[2], zmq_quat[3]});
        }

        const auto t_obs_start = std::chrono::steady_clock::now();
        MotionWindow window;
        try
        {
            window = zmq_motion_buffer->get_observation();
        }
        catch (const std::exception &error)
        {
            latch_super_fault("SuperMimic motion window", error.what());
            return;
        }
        const bool new_stream = window.reference_ready() && window.generation != zmq_last_generation;
        if (zmq_first_step || window.status != zmq_last_buffer_status || new_stream)
        {
            std::printf("[motion window] generation=%llu status=%s recv_age=%.1fms clamp=%d/%d\n",
                        static_cast<unsigned long long>(window.generation),
                        SuperRealtimeMotionBuffer::status_name(window.status),
                        window.latest_recv_age_ns / 1.e6, window.clamped_low, window.clamped_high);
        }
        // BUFFERING uses the configured standing reference, or the last consumed
        // reference after a reset. STALE holds one reference pose. Continue the
        // balance policy explicitly; neither status is treated as fresh motion.
        // Rebuild yaw, previous action and history once a NEW generation is ready.
        if (zmq_first_step || new_stream)
        {
            zmq_initialize_alignment(window.frames[0]);
            zmq_initialize_last_action();
            zmq_construct_single_obs();
            zmq_init_history();
        }
        else
        {
            zmq_construct_single_obs();
            zmq_push_history();
        }
        zmq_pack_history(window.frames);
        if (new_stream) zmq_last_generation = window.generation;
        zmq_last_buffer_status = window.status;
        zmq_first_step = false;
        const auto t_obs_end = std::chrono::steady_clock::now();

        if (!std::all_of(zmq_stacked_obs.begin(), zmq_stacked_obs.end(),
                         [](float value) { return std::isfinite(value); }))
        {
            latch_super_fault("SuperMimic observation", "nonfinite packed observation");
            return;
        }
        const auto t_infer_start = std::chrono::steady_clock::now();
        try
        {
            mMimicJakaMiniSuperZmqPolicy.session->Run(
                Ort::RunOptions(nullptr),
                &mMimicJakaMiniSuperZmqPolicy.input_name,
                mMimicJakaMiniSuperZmqPolicy.input_tensor,
                1,
                &mMimicJakaMiniSuperZmqPolicy.output_name,
                mMimicJakaMiniSuperZmqPolicy.output_tensor,
                1);
        }
        catch (const std::exception &error)
        {
            latch_super_fault("SuperMimic inference", error.what());
            return;
        }
        const auto t_infer_end = std::chrono::steady_clock::now();

        const auto t_action_start = std::chrono::steady_clock::now();
        if (!zmq_fill_trajectory())
        {
            latch_super_fault("SuperMimic action", "invalid policy action/target or ankle mapping");
            return;
        }
        if (mRobot->set_joint_trajectory(mJointNames, mPt) < 0)
        {
            latch_super_fault("SuperMimic trajectory", "controller rejected joint trajectory");
            return;
        }
        zmq_write_policy_diagnostic(window, new_stream || zmq_timing_step_count == 0);
        const auto t_action_end = std::chrono::steady_clock::now();

        // Calculate timings in milliseconds
        const double obs_ms = std::chrono::duration<double, std::milli>(t_obs_end - t_obs_start).count();
        const double infer_ms = std::chrono::duration<double, std::milli>(t_infer_end - t_infer_start).count();
        const double action_ms = std::chrono::duration<double, std::milli>(t_action_end - t_action_start).count();
        const double total_ms = std::chrono::duration<double, std::milli>(t_action_end - t_step_start).count();

        zmq_timing_step_count++;
        zmq_timing_obs_sum += obs_ms;
        zmq_timing_infer_sum += infer_ms;
        zmq_timing_action_sum += action_ms;
        zmq_timing_total_sum += total_ms;

        zmq_timing_obs_max = std::max(zmq_timing_obs_max, obs_ms);
        zmq_timing_infer_max = std::max(zmq_timing_infer_max, infer_ms);
        zmq_timing_action_max = std::max(zmq_timing_action_max, action_ms);
        zmq_timing_total_max = std::max(zmq_timing_total_max, total_ms);

        static int print_interval = -1;
        if (print_interval < 0)
        {
            const char *env_val = std::getenv("TIMING_INTERVAL");
            print_interval = (env_val != nullptr) ? std::atoi(env_val) : kDefaultTimingPrintInterval;
            if (print_interval <= 0)
                print_interval = kDefaultTimingPrintInterval;
        }

        const bool is_initial_steps = (zmq_timing_step_count <= 5);
        const bool is_periodic_step = (zmq_timing_step_count % print_interval == 0);
        const bool is_spike = (total_ms > kLatencySpikeThresholdMs);

        if (is_spike)
        {
            printf("[superzmq SPIKE] step: %lu | obs: %.3f ms | infer: %.3f ms | action: %.3f ms | total: %.3f ms (> %.1f ms!)\n",
                   static_cast<unsigned long>(zmq_timing_step_count),
                   obs_ms, infer_ms, action_ms, total_ms, kLatencySpikeThresholdMs);
        }
        else if (is_initial_steps || is_periodic_step)
        {
            const double avg_obs = zmq_timing_obs_sum / zmq_timing_step_count;
            const double avg_infer = zmq_timing_infer_sum / zmq_timing_step_count;
            const double avg_action = zmq_timing_action_sum / zmq_timing_step_count;
            const double avg_total = zmq_timing_total_sum / zmq_timing_step_count;

            printf("[superzmq timing] step: %lu | obs: %.3f ms (avg: %.3f, max: %.3f) | infer: %.3f ms (avg: %.3f, max: %.3f) | action: %.3f ms (avg: %.3f, max: %.3f) | total: %.3f ms (avg: %.3f, max: %.3f)\n",
                   static_cast<unsigned long>(zmq_timing_step_count),
                   obs_ms, avg_obs, zmq_timing_obs_max,
                   infer_ms, avg_infer, zmq_timing_infer_max,
                   action_ms, avg_action, zmq_timing_action_max,
                   total_ms, avg_total, zmq_timing_total_max);
        }
    }

    void FSMMimicJakaMiniSuperZmqEnter(FSMSTATE from)
    {
        if (from != FSM_IDLE)
        {
            mErrCode = ERR_STATE_TRANSITION;
            if (mRobot) mRobot->do_enable(false);
            return;
        }
        reset_super_parallel_ankles();
        if (!zmq_resolve_policy_layout())
        {
            latch_super_fault("SuperMimic entry", "invalid policy layout");
            mErrCode = ERR_INIT;
            return;
        }

        mFSMState = ReqState = FSM_JAKAMINIMIMIC_SUPERZMQ;
        StateJudge = &fsm_mimic_jaka_mini_superzmq_judge;
        StateRun = &fsm_mimic_jaka_mini_superzmq_run;
        zmq_exit_chord_released = false;
        mRobot->set_cur_control_stat(FSM_JAKAMINIMIMIC_SUPERZMQ);
        zmq_first_step = true;
        zmq_last_generation = 0;
        zmq_last_buffer_status = MotionWindow::Status::Buffering;
        zmq_policy_diagnostic.close();
        zmq_policy_diagnostic.clear();
        if (const char *path = std::getenv("JAKA_SUPER_POLICY_DIAG_JSONL"); path && *path)
        {
            zmq_policy_diagnostic.open(path);
            if (!zmq_policy_diagnostic)
            {
                latch_super_fault("SuperMimic entry", "cannot open policy diagnostic tape");
                return;
            }
        }

        zmq_timing_step_count = 0;
        zmq_timing_obs_sum = 0.0;
        zmq_timing_infer_sum = 0.0;
        zmq_timing_action_sum = 0.0;
        zmq_timing_total_sum = 0.0;
        zmq_timing_obs_max = 0.0;
        zmq_timing_infer_max = 0.0;
        zmq_timing_action_max = 0.0;
        zmq_timing_total_max = 0.0;

        if (mDebugFlag)
            printf("--------from state %d to jaka mini mimic ZMQ state %d------------\n",
                   from, FSM_JAKAMINIMIMIC_SUPERZMQ);

        if (!zmq_fetch_robot_state())
        {
            mErrCode = ERR_JOINT;
            return;
        }

        try
        {
            zmq_motion_buffer = std::make_unique<SuperRealtimeMotionBuffer>(
                mJointNames,
                mJakaMiniSuperZmqConnect,
                mJakaMiniSuperZmqHwm,
                static_cast<int64_t>(mJakaMiniSuperZmqDt * 1e9),
                static_cast<int64_t>(mJakaMiniSuperZmqTolerance * 1e9),
                mJakaMiniSuperZmqFutureSteps,
                mJakaMiniSuperZmqDefaultJointPos.size() > 0 ? mJakaMiniSuperZmqDefaultJointPos
                                                       : Eigen::VectorXf::Zero(mNumAction),
                mJakaMiniSuperZmqDefaultRootPos,
                mJakaMiniSuperZmqDefaultRootQuat,
                mJakaMiniSuperZmqTiming,
                0, static_cast<float>(mJakaMiniSuperZmqRootHeightOffset));

            // OpenHLM 风格 state 发布器(绑定 :28711; 失败仅告警, 不影响控制)
            zmq_state_pub = std::make_unique<StatePublisher>(mStateZmqBind);
        }
        catch (const std::exception &error)
        {
            zmq_motion_buffer.reset();
            zmq_state_pub.reset();
            latch_super_fault("SuperMimic entry", error.what());
            return;
        }

        switch (from)
        {
        case FSM_IDLE:
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }

    static void from_fsm_mimic_jaka_mini_superzmq()
    {
        zmq_motion_buffer.reset();
        zmq_state_pub.reset();
        zmq_policy_diagnostic.close();
        switch (ReqState)
        {
        case FSM_IDLE:
            FSMIdleEnter(FSM_JAKAMINIMIMIC_SUPERZMQ);
            break;
        case FSM_FAULT:
            FSMFaultEnter(FSM_JAKAMINIMIMIC_SUPERZMQ);
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }

} // namespace RLController
