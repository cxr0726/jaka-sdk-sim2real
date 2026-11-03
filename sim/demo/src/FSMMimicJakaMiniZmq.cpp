#include "RLController.hpp"
#include "StatePublisher.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace RLController
{

    extern fsm_func_ptr StateJudge;
    extern fsm_func_ptr StateRun;
    extern FSMSTATE ReqState;

    namespace
    {
        constexpr int kJakaMiniZMQCommandDim = 155;
        constexpr int kJakaMiniZMQAnchorOriDim = 30;
        constexpr int kJakaMiniZMQGravityDim = 3;
        constexpr int kJakaMiniZMQAngVelDim = 3;
        constexpr int kJakaMiniZMQHistObsDim = 87;
        constexpr int kJakaMiniZMQFutureSteps = 5;
        constexpr int kJakaMiniZMQRot6dStride = 6;
        constexpr int kJakaMiniZMQDefaultFrameStack = 5;
        constexpr float kJakaMiniZMQPi = 3.14159265358979323846f;
        constexpr std::array<int, 27> kJakaMiniZMQIsaaclabToDeployIndex = {
            0, 12, 24,
            1, 13, 6, 25, 18,
            2, 14, 7, 26, 19,
            3, 15, 8, 20,
            4, 16, 9, 21,
            5, 17, 10, 22,
            11, 23};

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
    static bool zmq_play;
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

    static std::unique_ptr<RealtimeMotionBuffer> zmq_motion_buffer;

    // OpenHLM 风格 state 发布器(50Hz tick 内联、非阻塞; 部署序→JAKA 序重排)
    static std::unique_ptr<StatePublisher> zmq_state_pub;

    static void from_fsm_mimic_jaka_mini_zmq();

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

    static inline std::array<float, kJakaMiniZMQRot6dStride> zmq_quat_to_rot6d(const Eigen::Quaternionf &q)
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
        if (mMimicJakaMiniZmqPolicy.session == nullptr)
        {
            std::cerr << "[FSMJakaMiniZmq] policy session is null." << std::endl;
            return false;
        }

        zmq_frame_stack = kJakaMiniZMQDefaultFrameStack;
        if (mMimicJakaMiniZmqPolicy.input_shape[1] > 0)
        {
            const int non_history_dim = kJakaMiniZMQCommandDim + kJakaMiniZMQAnchorOriDim;
            const int remaining = static_cast<int>(mMimicJakaMiniZmqPolicy.input_shape[1]) - non_history_dim;
            if (remaining <= 0 || remaining % kJakaMiniZMQHistObsDim != 0)
            {
                std::cerr << "[FSMJakaMiniZmq] input dim mismatch" << std::endl;
                return false;
            }
            zmq_frame_stack = remaining / kJakaMiniZMQHistObsDim;
        }

        if (static_cast<int>(mMimicJakaMiniZmqPolicy.output_shape[1]) != mNumAction)
        {
            std::cerr << "[FSMJakaMiniZmq] output dim mismatch" << std::endl;
            return false;
        }

        zmq_single_obs.assign(kJakaMiniZMQHistObsDim, 0.0f);
        zmq_stacked_obs.assign(static_cast<int>(mMimicJakaMiniZmqPolicy.input_shape[1]), 0.0f);
        zmq_last_action.assign(mNumAction, 0.0f);
        zmq_obs_history.clear();

        std::cerr << "[FSMJakaMiniZmq] action_dim=" << mNumAction
                  << ", frame_stack=" << zmq_frame_stack << std::endl;
        return true;
    }

    static inline bool zmq_fetch_robot_state()
    {
        update_joint_status(zmq_q_cmd, zmq_q, zmq_qd);
        if (mEnabledJointCount != mNumAction || mFaultJointCount > 0)
            return false;

        if (!decode_parallel_ankles(zmq_q, zmq_qd, "mimic ZMQ feedback"))
            return false;

        if (mRobot->get_imu(zmq_imu))
        {
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

            // // Transform IMU readings from base_link to waist_yaw_link using forward kinematics via waist_yaw_joint (index 24)
            // {
            //     const double theta = zmq_q[24];
            //     const double dtheta = zmq_qd[24];

            //     // 1. Transform orientation quaternion: q_W = q_B * q_W_B
            //     Eigen::Quaternionf q_B(
            //         static_cast<float>(zmq_quat[0]),
            //         static_cast<float>(zmq_quat[1]),
            //         static_cast<float>(zmq_quat[2]),
            //         static_cast<float>(zmq_quat[3])
            //     );
            //     Eigen::Quaternionf q_W_B(
            //         static_cast<float>(std::cos(theta / 2.0)),
            //         0.0f,
            //         0.0f,
            //         static_cast<float>(std::sin(theta / 2.0))
            //     );
            //     Eigen::Quaternionf q_W = q_B * q_W_B;
            //     q_W.normalize();

            //     zmq_quat[0] = q_W.w();
            //     zmq_quat[1] = q_W.x();
            //     zmq_quat[2] = q_W.y();
            //     zmq_quat[3] = q_W.z();

            //     // 2. Transform angular velocity: omega_W = R_z(-theta) * omega_B + [0, 0, dtheta]^T
            //     const double cos_theta = std::cos(theta);
            //     const double sin_theta = std::sin(theta);
            //     const double omega_W_x = cos_theta * zmq_omega[0] + sin_theta * zmq_omega[1];
            //     const double omega_W_y = -sin_theta * zmq_omega[0] + cos_theta * zmq_omega[1];
            //     const double omega_W_z = zmq_omega[2] + dtheta;

            //     zmq_omega[0] = omega_W_x;
            //     zmq_omega[1] = omega_W_y;
            //     zmq_omega[2] = omega_W_z;
            // }
        }
        else
        {
            zmq_acc[0] = zmq_acc[1] = zmq_acc[2] = 0.0;
            zmq_omega[0] = zmq_omega[1] = zmq_omega[2] = 0.0;
            zmq_quat[0] = 1.0;
            zmq_quat[1] = zmq_quat[2] = zmq_quat[3] = 0.0;
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
        for (int isaac_idx = 0; isaac_idx < mNumAction; ++isaac_idx)
        {
            const int deploy_idx = kJakaMiniZMQIsaaclabToDeployIndex[isaac_idx];
            zmq_last_action[isaac_idx] =
                static_cast<float>((zmq_q_cmd[deploy_idx] - mDefaultPos[deploy_idx]) / mActionScale);
        }
    }

    static inline std::array<float, kJakaMiniZMQCommandDim> zmq_construct_command(
        const std::array<InterpolatedFrame, RealtimeMotionBuffer::kFutureSteps> &frames)
    {
        std::array<float, kJakaMiniZMQCommandDim> cmd{};
        const Eigen::Vector3f anchor_pos = frames[0].root_pos_w;
        const Eigen::Quaternionf anchor_quat = zmq_normalize_quat(frames[0].root_quat_w);

        int offset = 0;
        for (int i = 0; i < kJakaMiniZMQFutureSteps; ++i)
        {
            const Eigen::Vector3f diff_w = frames[i].root_pos_w - anchor_pos;
            const Eigen::Vector3f diff_b = zmq_quat_rotate_inverse(anchor_quat, diff_w);
            cmd[offset++] = diff_b[0];
            cmd[offset++] = diff_b[1];
            cmd[offset++] = diff_b[2];
        }

        for (int i = 0; i < kJakaMiniZMQFutureSteps; ++i)
            cmd[offset++] = frames[i].root_pos_w[2];

        for (int i = 0; i < kJakaMiniZMQFutureSteps; ++i)
        {
            for (int isaac_idx = 0; isaac_idx < mNumAction; ++isaac_idx)
            {
                float jv = 0.0f;
                if (isaac_idx < frames[i].dof_pos.size())
                    jv = frames[i].dof_pos[isaac_idx];
                cmd[offset++] = jv;
            }
        }

        return cmd;
    }

    static inline std::array<float, kJakaMiniZMQAnchorOriDim> zmq_construct_anchor_orientation(
        const std::array<InterpolatedFrame, RealtimeMotionBuffer::kFutureSteps> &frames)
    {
        const Eigen::Quaternionf robot_quat_now = zmq_robot_quat_from_imu();
        std::array<float, kJakaMiniZMQAnchorOriDim> ori{};

        int offset = 0;
        for (int i = 0; i < kJakaMiniZMQFutureSteps; ++i)
        {
            const Eigen::Quaternionf ref_quat = zmq_normalize_quat(frames[i].root_quat_w);
            const Eigen::Quaternionf anchor_world = zmq_normalize_quat(zmq_align_quat * ref_quat);
            const Eigen::Quaternionf ori_body = zmq_normalize_quat(robot_quat_now.conjugate() * anchor_world);
            const auto rot6d = zmq_quat_to_rot6d(ori_body);
            for (int j = 0; j < kJakaMiniZMQRot6dStride; ++j)
                ori[offset++] = rot6d[j];
        }
        return ori;
    }

    static inline std::array<float, kJakaMiniZMQGravityDim> zmq_construct_gravity()
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
        off += kJakaMiniZMQGravityDim;

        for (int i = 0; i < kJakaMiniZMQAngVelDim; ++i)
            zmq_single_obs[off + i] = static_cast<float>(zmq_omega[i] * 0.25);
        off += kJakaMiniZMQAngVelDim;

        for (int isaac_idx = 0; isaac_idx < mNumAction; ++isaac_idx)
        {
            const int deploy_idx = kJakaMiniZMQIsaaclabToDeployIndex[isaac_idx];
            zmq_single_obs[off + isaac_idx] = static_cast<float>(zmq_q[deploy_idx] - mDefaultPos[deploy_idx]);
        }
        off += mNumAction;

        for (int isaac_idx = 0; isaac_idx < mNumAction; ++isaac_idx)
        {
            const int deploy_idx = kJakaMiniZMQIsaaclabToDeployIndex[isaac_idx];
            zmq_single_obs[off + isaac_idx] = static_cast<float>(zmq_qd[deploy_idx] * 0.05);
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
        const std::array<InterpolatedFrame, RealtimeMotionBuffer::kFutureSteps> &frames)
    {
        const auto command = zmq_construct_command(frames);
        const auto anchor_ori = zmq_construct_anchor_orientation(frames);

        static constexpr std::array<int, 5> hist_blocks = {3, 3, 27, 27, 27};

        std::fill(zmq_stacked_obs.begin(), zmq_stacked_obs.end(), 0.0f);

        int w = 0;
        std::copy(command.begin(), command.end(), zmq_stacked_obs.begin() + w);
        w += kJakaMiniZMQCommandDim;
        std::copy(anchor_ori.begin(), anchor_ori.end(), zmq_stacked_obs.begin() + w);
        w += kJakaMiniZMQAnchorOriDim;

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
                  mMimicJakaMiniZmqPolicy.input_buffer);
    }

    static inline double zmq_clamp_target(double target, int idx)
    {
        return std::max(mDownLmt[idx], std::min(mUpLmt[idx], target));
    }

    static inline bool zmq_fill_trajectory()
    {
        for (int i = 0; i < mNumAction; ++i)
            if (!std::isfinite(mMimicJakaMiniZmqPolicy.output_buffer[i]))
                return latch_ankle_fault("mimic ZMQ policy output", "nonfinite action");
        for (int isaac_idx = 0; isaac_idx < mNumAction; ++isaac_idx)
        {
            const int deploy_idx = kJakaMiniZMQIsaaclabToDeployIndex[isaac_idx];
            zmq_last_action[isaac_idx] =
                std::clamp(mMimicJakaMiniZmqPolicy.output_buffer[isaac_idx], -5.0f, 5.0f);
            mPt.positions[deploy_idx] =
                static_cast<double>(zmq_last_action[isaac_idx]) * mActionScale + mDefaultPos[deploy_idx];
        }

        for (double target : mPt.positions)
            if (!std::isfinite(target))
                return latch_ankle_fault("mimic ZMQ target", "nonfinite target");

        if (mAngleMapFlag && !encode_parallel_ankle_targets(mPt.positions, "mimic ZMQ target"))
            return false;

        for (int i = 0; i < mNumAction; ++i)
            mPt.positions[i] = zmq_clamp_target(mPt.positions[i], i);

        mPt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(mTimeStep));
        return true;
    }

    static void fsm_mimic_jaka_mini_zmq_judge()
    {
        mRobot->get_joy_robot_state(zmq_joydata);
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

            if (!zmq_play && zmq_joydata.joy.buttons_RB)
            {
                zmq_play = true;
            }
        }

        if (ReqState != FSM_JAKAMINIMIMIC_ZMQ)
        {
            from_fsm_mimic_jaka_mini_zmq();
        }
    }

    static void fsm_mimic_jaka_mini_zmq_run()
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
        const MotionWindow window = zmq_motion_buffer->get_observation();
        const bool new_stream = window.reference_ready() && window.generation != zmq_last_generation;
        if (zmq_first_step || window.status != zmq_last_buffer_status || new_stream)
        {
            std::printf("[motion window] generation=%llu status=%s recv_age=%.1fms clamp=%d/%d\n",
                        static_cast<unsigned long long>(window.generation),
                        RealtimeMotionBuffer::status_name(window.status),
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

        const auto t_infer_start = std::chrono::steady_clock::now();
        mMimicJakaMiniZmqPolicy.session->Run(
            Ort::RunOptions(nullptr),
            &mMimicJakaMiniZmqPolicy.input_name,
            mMimicJakaMiniZmqPolicy.input_tensor,
            1,
            &mMimicJakaMiniZmqPolicy.output_name,
            mMimicJakaMiniZmqPolicy.output_tensor,
            1);
        const auto t_infer_end = std::chrono::steady_clock::now();

        const auto t_action_start = std::chrono::steady_clock::now();
        if (!zmq_fill_trajectory()) return;
        mRobot->set_joint_trajectory(mJointNames, mPt);
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
            printf("[zmq SPIKE] step: %lu | obs: %.3f ms | infer: %.3f ms | action: %.3f ms | total: %.3f ms (> %.1f ms!)\n",
                   static_cast<unsigned long>(zmq_timing_step_count),
                   obs_ms, infer_ms, action_ms, total_ms, kLatencySpikeThresholdMs);
        }
        else if (is_initial_steps || is_periodic_step)
        {
            const double avg_obs = zmq_timing_obs_sum / zmq_timing_step_count;
            const double avg_infer = zmq_timing_infer_sum / zmq_timing_step_count;
            const double avg_action = zmq_timing_action_sum / zmq_timing_step_count;
            const double avg_total = zmq_timing_total_sum / zmq_timing_step_count;

            printf("[zmq timing] step: %lu | obs: %.3f ms (avg: %.3f, max: %.3f) | infer: %.3f ms (avg: %.3f, max: %.3f) | action: %.3f ms (avg: %.3f, max: %.3f) | total: %.3f ms (avg: %.3f, max: %.3f)\n",
                   static_cast<unsigned long>(zmq_timing_step_count),
                   obs_ms, avg_obs, zmq_timing_obs_max,
                   infer_ms, avg_infer, zmq_timing_infer_max,
                   action_ms, avg_action, zmq_timing_action_max,
                   total_ms, avg_total, zmq_timing_total_max);
        }
    }

    void FSMMimicJakaMiniZmqEnter(FSMSTATE from)
    {
        if (!zmq_resolve_policy_layout())
        {
            mErrCode = ERR_INIT;
            return;
        }

        mFSMState = ReqState = FSM_JAKAMINIMIMIC_ZMQ;
        StateJudge = &fsm_mimic_jaka_mini_zmq_judge;
        StateRun = &fsm_mimic_jaka_mini_zmq_run;
        zmq_play = false;
        zmq_first_step = true;
        zmq_last_generation = 0;
        zmq_last_buffer_status = MotionWindow::Status::Buffering;

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
                   from, FSM_JAKAMINIMIMIC_ZMQ);

        if (!zmq_fetch_robot_state())
        {
            mErrCode = ERR_JOINT;
            return;
        }

        zmq_motion_buffer = std::make_unique<RealtimeMotionBuffer>(
            mJointNames,
            mJakaMiniZmqConnect,
            mJakaMiniZmqHwm,
            static_cast<int64_t>(mJakaMiniZmqDt * 1e9),
            static_cast<int64_t>(mJakaMiniZmqTolerance * 1e9),
            mJakaMiniZmqFutureSteps,
            mJakaMiniZmqDefaultJointPos.size() > 0 ? mJakaMiniZmqDefaultJointPos
                                                   : Eigen::VectorXf::Zero(mNumAction),
            mJakaMiniZmqDefaultRootPos,
            mJakaMiniZmqDefaultRootQuat,
            mJakaMiniZmqTiming,
            mJakaMiniZmqReferenceBodyIndex);

        // OpenHLM 风格 state 发布器(绑定 :28711; 失败仅告警, 不影响控制)
        zmq_state_pub = std::make_unique<StatePublisher>(mStateZmqBind);

        switch (from)
        {
        case FSM_IDLE:
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }

    static void from_fsm_mimic_jaka_mini_zmq()
    {
        zmq_motion_buffer.reset();
        zmq_state_pub.reset();
        switch (ReqState)
        {
        case FSM_IDLE:
            FSMIdleEnter(FSM_JAKAMINIMIMIC_ZMQ);
            break;
        case FSM_LOCO:
            FSMLocoEnter(FSM_JAKAMINIMIMIC_ZMQ);
            break;
        case FSM_MIMIC:
            FSMMimicEnter(FSM_JAKAMINIMIMIC_ZMQ);
            break;
        case FSM_FAULT:
            FSMFaultEnter(FSM_JAKAMINIMIMIC_ZMQ);
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }

} // namespace RLController
