// Khan SuperMimic packed actor deployment. Deliberately follows the existing
// FSMMimicJakaMiniZmq control/observation/history structure; joint and scale
// semantics are read from this model's embedded deployment metadata.
#include "RLController.hpp"
#include "StatePublisher.hpp"
#include "LegacyRealtimeMotionBuffer.hpp"

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
    static bool zmq_play;
    static bool zmq_first_step;
    static bool zmq_last_ready;
    static int zmq_frame_stack;
    static Eigen::Quaternionf zmq_align_quat(1.0f, 0.0f, 0.0f, 0.0f);
    static std::vector<float> zmq_single_obs;
    static std::vector<float> zmq_stacked_obs;
    static std::vector<float> zmq_last_action;
    static std::vector<double> zmq_raw_targets;
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

    static std::unique_ptr<LegacyRealtimeMotionBuffer> zmq_motion_buffer;

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
        update_joint_status(zmq_q_cmd, zmq_q, zmq_qd);
        if (mEnabledJointCount != mNumAction || mFaultJointCount > 0)
            return false;

        if (!decode_parallel_ankles(zmq_q, zmq_qd, "SuperMimic ZMQ feedback"))
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

            // SuperMimic is trained on base_link orientation and body angular
            // velocity. imu_process pairs with the SDK sensor-axis compensation;
            // do not append the legacy waist theta/dtheta transform here.
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
        // Training reset contract: previous RAW policy action is zero.
        std::fill(zmq_last_action.begin(), zmq_last_action.end(), 0.0f);
    }

    static inline std::array<float, kJakaMiniSuperZMQCommandDim> zmq_construct_command(
        const std::array<InterpolatedFrame, LegacyRealtimeMotionBuffer::kFutureSteps> &frames)
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
        const std::array<InterpolatedFrame, LegacyRealtimeMotionBuffer::kFutureSteps> &frames)
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
        const std::array<InterpolatedFrame, LegacyRealtimeMotionBuffer::kFutureSteps> &frames)
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
        // Preserve raw actions/history. Optional physical PR target clipping
        // reproduces the real YAML control law for an explicit simulation A/B.
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

        zmq_raw_targets = mPt.positions;
        if (mJakaMiniSuperZmqClipTargetPositions)
        {
            if (mAngleMapFlag != 0) return false;
            for (int deploy = 0; deploy < mNumAction; ++deploy)
                mPt.positions[deploy] = std::clamp(mPt.positions[deploy], mDownLmt[deploy], mUpLmt[deploy]);
        }

        if (mAngleMapFlag && !encode_parallel_ankle_targets(mPt.positions, "SuperMimic ZMQ target"))
            return false;
        mPt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(mTimeStep));
        return std::all_of(mPt.positions.begin(), mPt.positions.end(),
                           [](double value) { return std::isfinite(value); });
    }

    static void zmq_write_policy_diagnostic(
        const std::array<InterpolatedFrame, LegacyRealtimeMotionBuffer::kFutureSteps> &sampled_frames,
        bool reset_history, bool buffer_ready = false, bool reset_alignment = false)
    {
        if (!zmq_policy_diagnostic.is_open()) return;
        nlohmann::json frames = nlohmann::json::array();
        for (const auto &frame : sampled_frames)
            frames.push_back({{"root_pos", {frame.root_pos_w.x(), frame.root_pos_w.y(), frame.root_pos_w.z()}},
                              {"root_quat", {frame.root_quat_w.w(), frame.root_quat_w.x(), frame.root_quat_w.y(), frame.root_quat_w.z()}},
                              {"joint_pos_buffer", std::vector<float>(frame.dof_pos.data(), frame.dof_pos.data() + frame.dof_pos.size())}});
        const nlohmann::json record = {
            {"step", zmq_timing_step_count},
            {"buffer_backend", "legacy_arrival"}, {"reference_ready", buffer_ready},
            {"query_steady_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count()},
            {"query_time_scope", "diagnostic_write_not_buffer_query"},
            // Compatibility fields for existing offline policy-tape readers.
            // The legacy buffer has no stream generations or stale status.
            {"generation", buffer_ready ? 1 : 0}, {"status", buffer_ready ? "tracking" : "buffering"},
            {"reset_history", reset_history}, {"reset_alignment", reset_history || reset_alignment}, {"frames", frames},
            {"robot_quat", {zmq_quat[0], zmq_quat[1], zmq_quat[2], zmq_quat[3]}},
            {"omega", {zmq_omega[0], zmq_omega[1], zmq_omega[2]}},
            {"q_deploy", std::vector<double>(zmq_q, zmq_q + mNumAction)},
            {"qd_deploy", std::vector<double>(zmq_qd, zmq_qd + mNumAction)},
            {"align_quat", {zmq_align_quat.w(), zmq_align_quat.x(), zmq_align_quat.y(), zmq_align_quat.z()}},
            {"obs", zmq_stacked_obs}, {"actions", zmq_last_action},
            {"clip_target_positions", mJakaMiniSuperZmqClipTargetPositions},
            {"targets_raw_deploy", zmq_raw_targets}, {"targets_deploy", mPt.positions}};
        zmq_policy_diagnostic << record.dump() << '\n';
    }

    static void fsm_mimic_jaka_mini_superzmq_judge()
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
        const bool buffer_ready = zmq_motion_buffer->ready();
        const auto frames = zmq_motion_buffer->get_obs();
        const bool reset_history = zmq_first_step;
        const bool reset_alignment = zmq_first_step || (!zmq_last_ready && buffer_ready);
        if (zmq_first_step)
        {
            zmq_initialize_alignment(frames[0]);
            zmq_initialize_last_action();
            zmq_construct_single_obs();
            zmq_init_history();
        }
        else
        {
            // Original real ZMQ behavior: first received reference realigns yaw
            // while retaining the running action/sensor history.
            if (reset_alignment) zmq_initialize_alignment(frames[0]);
            zmq_construct_single_obs();
            zmq_push_history();
        }
        zmq_pack_history(frames);
        zmq_last_ready = buffer_ready;
        zmq_first_step = false;
        const auto t_obs_end = std::chrono::steady_clock::now();

        const auto t_infer_start = std::chrono::steady_clock::now();
        mMimicJakaMiniSuperZmqPolicy.session->Run(
            Ort::RunOptions(nullptr),
            &mMimicJakaMiniSuperZmqPolicy.input_name,
            mMimicJakaMiniSuperZmqPolicy.input_tensor,
            1,
            &mMimicJakaMiniSuperZmqPolicy.output_name,
            mMimicJakaMiniSuperZmqPolicy.output_tensor,
            1);
        const auto t_infer_end = std::chrono::steady_clock::now();

        const auto t_action_start = std::chrono::steady_clock::now();
        if (!zmq_fill_trajectory())
        {
            std::cerr << "[FSMJakaMiniSuperZmq] invalid policy action/target or ankle mapping" << std::endl;
            mErrCode = ERR_JOINT;
            return;
        }
        mRobot->set_joint_trajectory(mJointNames, mPt);
        zmq_write_policy_diagnostic(frames, reset_history, buffer_ready, reset_alignment);
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
        if (!zmq_resolve_policy_layout())
        {
            mErrCode = ERR_INIT;
            return;
        }

        mFSMState = ReqState = FSM_JAKAMINIMIMIC_SUPERZMQ;
        StateJudge = &fsm_mimic_jaka_mini_superzmq_judge;
        StateRun = &fsm_mimic_jaka_mini_superzmq_run;
        zmq_play = false;
        zmq_first_step = true;
        zmq_last_ready = false;
        zmq_policy_diagnostic.close();
        zmq_policy_diagnostic.clear();
        if (const char *path = std::getenv("JAKA_SUPER_POLICY_DIAG_JSONL"); path && *path)
        {
            zmq_policy_diagnostic.open(path);
            if (!zmq_policy_diagnostic)
                throw std::runtime_error("Cannot open SuperMimic policy diagnostic tape");
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

        zmq_motion_buffer = std::make_unique<LegacyRealtimeMotionBuffer>(
            mJointNames,
            mJakaMiniSuperZmqConnect,
            mJakaMiniSuperZmqHwm,
            static_cast<int64_t>(mJakaMiniSuperZmqDt * 1e9),
            static_cast<int64_t>(mJakaMiniSuperZmqTolerance * 1e9),
            mJakaMiniSuperZmqFutureSteps,
            mJakaMiniSuperZmqDefaultJointPos.size() > 0 ? mJakaMiniSuperZmqDefaultJointPos
                                                   : Eigen::VectorXf::Zero(mNumAction),
            mJakaMiniSuperZmqDefaultRootPos,
            mJakaMiniSuperZmqDefaultRootQuat);

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
        case FSM_LOCO:
            FSMLocoEnter(FSM_JAKAMINIMIMIC_SUPERZMQ);
            break;
        case FSM_MIMIC:
            FSMMimicEnter(FSM_JAKAMINIMIMIC_SUPERZMQ);
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
