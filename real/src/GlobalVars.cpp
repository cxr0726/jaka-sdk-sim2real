#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <array>
#include <sstream>
#include <set>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include "RLController.hpp"

namespace RLController
{
    namespace
    {
        void validate_motion_sampling(double dt, double tolerance, const std::vector<int> &steps)
        {
            // Bound before the FSM converts seconds to int64 nanoseconds.
            if (!std::isfinite(dt) || dt <= 0 || dt > 1 ||
                !std::isfinite(tolerance) || tolerance < 0 || tolerance > 60 || steps.size() != 5)
                throw std::runtime_error("Invalid motion dt/tolerance/future_steps");
            for (size_t i = 0; i < steps.size(); ++i)
                if (steps[i] < -10000 || steps[i] > 10000 || (i && steps[i] <= steps[i - 1]))
                    throw std::runtime_error("Invalid motion future_steps range/order");
        }

        MotionTimingConfig load_motion_timing(const YAML::Node &config)
        {
            MotionTimingConfig result;
            const auto clock = config["clock"];
            if (!clock) return result;
            const std::string mode = clock["timestamp_mode"] ? clock["timestamp_mode"].as<std::string>() : "source";
            if (mode == "source") result.timestamp_mode = MotionTimingConfig::TimestampMode::Source;
            else if (mode == "publish") result.timestamp_mode = MotionTimingConfig::TimestampMode::Publish;
            else if (mode == "arrival") result.timestamp_mode = MotionTimingConfig::TimestampMode::Arrival;
            else throw std::runtime_error("motion clock.timestamp_mode must be source, publish or arrival");
            auto read_ns = [&](const char *key, int64_t fallback) {
                if (!clock[key]) return fallback;
                const double seconds = clock[key].as<double>();
                if (!std::isfinite(seconds) || seconds < 0 || seconds > 3600)
                    throw std::runtime_error(std::string("Invalid motion clock.") + key);
                return static_cast<int64_t>(std::llround(seconds * 1.e9));
            };
            result.clock.warmup_ns = read_ns("warmup_s", result.clock.warmup_ns);
            result.clock.window_ns = read_ns("statistics_window_s", result.clock.window_ns);
            if (clock["min_samples"]) result.clock.min_samples = clock["min_samples"].as<size_t>();
            if (clock["quantile"]) result.clock.quantile = clock["quantile"].as<double>();
            if (clock["max_slew"]) result.clock.max_slew = clock["max_slew"].as<double>();
            result.stale_timeout_ns = read_ns("stale_timeout_s", result.stale_timeout_ns);
            result.reset_gap_ns = read_ns("reset_gap_s", result.reset_gap_ns);
            result.discontinuity_ns = read_ns("discontinuity_s", result.discontinuity_ns);
            // Validate mapper parameters during config loading, before starting a thread.
            SourceTimeline validate(result.clock);
            return result;
        }
    }
    // global vars definition
    std::unique_ptr<legged_waist_controller::RobotIntf> mRobot;
    trajectory_msgs::msg::JointTrajectoryPoint mPt;

    bool mRLThreadRunFlag = true;
    bool mDataReadyFlag = false;
    int mEnabledJointCount = 0;
    int mFaultJointCount = 0;

    FSMSTATE mFSMState = FSM_INIT;
    ERRCODE mErrCode = ERR_NONE;
    bool mDebugFlag = true;

    std::vector<double> mKp;
    std::vector<double> mKd;

    std::vector<double> mKp_PR;
    std::vector<double> mKd_PR;

    std::vector<std::string> mJointNames;

    int mAngleMapFlag;

    // yaml configs
    YAML::Node mYamlNode;

    OnnxCtx mLocoPolicy;
    int mNumSingleObs;
    int mNumFrameStack;
    int mNumAction;
    double mTimeStep;

    OnnxCtx mMimicPolicy;
    int mNumSingleObsMimic;
    int mNumFrameStackMimic;

    RefMotion mRefMotion;

    // jaka-mini mimic ZMQ settings
    OnnxCtx mMimicJakaMiniZmqPolicy;
    bool mJakaMiniZmqEnabled = false;
    std::string mJakaMiniZmqConnect = "tcp://127.0.0.1:28701";
    int mJakaMiniZmqHwm = 1;
    double mJakaMiniZmqDt = 0.02;
    double mJakaMiniZmqTolerance = 0.04;
    std::vector<int> mJakaMiniZmqFutureSteps;
    Eigen::VectorXf mJakaMiniZmqDefaultJointPos;
    Eigen::Vector3f mJakaMiniZmqDefaultRootPos = Eigen::Vector3f::Zero();
    Eigen::Quaternionf mJakaMiniZmqDefaultRootQuat = Eigen::Quaternionf::Identity();

    // jaka-mini mimic ZMQ fixwrist settings
    OnnxCtx mMimicJakaMiniZmqFixwristPolicy;
    bool mJakaMiniZmqFixwristEnabled = false;
    std::string mJakaMiniZmqFixwristConnect = "tcp://127.0.0.1:28701";
    int mJakaMiniZmqFixwristHwm = 1;
    double mJakaMiniZmqFixwristDt = 0.02;
    double mJakaMiniZmqFixwristTolerance = 0.04;
    std::vector<int> mJakaMiniZmqFixwristFutureSteps;
    Eigen::VectorXf mJakaMiniZmqFixwristDefaultJointPos;
    Eigen::Vector3f mJakaMiniZmqFixwristDefaultRootPos = Eigen::Vector3f::Zero();
    Eigen::Quaternionf mJakaMiniZmqFixwristDefaultRootQuat = Eigen::Quaternionf::Identity();

    OnnxCtx mMimicJakaMiniSuperZmqPolicy;
    bool mJakaMiniSuperZmqEnabled = false;
    std::string mJakaMiniSuperZmqConnect = "tcp://10.5.5.120:28701";
    int mJakaMiniSuperZmqHwm = 1;
    double mJakaMiniSuperZmqDt = 0.02;
    double mJakaMiniSuperZmqTolerance = 0.04;
    std::vector<int> mJakaMiniSuperZmqFutureSteps;
    Eigen::VectorXf mJakaMiniSuperZmqDefaultJointPos;
    Eigen::Vector3f mJakaMiniSuperZmqDefaultRootPos(0.0f, 0.0f, 0.65f);
    Eigen::Quaternionf mJakaMiniSuperZmqDefaultRootQuat = Eigen::Quaternionf::Identity();
    MotionTimingConfig mJakaMiniSuperZmqTiming;
    double mJakaMiniSuperZmqRootHeightOffset = 0.05;
    std::unique_ptr<SuperMimicDeployment> mJakaMiniSuperDeployment;

    // OpenHLM 风格 state 发布(ZMQ PUB, JAKA 关节序, 控制 tick 内非阻塞)
    std::string mStateZmqBind = "tcp://*:28711";

    double mActionScale;
    double mDofPosScale;
    double mDofVelScale;
    double mOmegaScale;

    std::vector<double> mDefaultPos;
    std::vector<double> mInitTarget;
    std::vector<double> mZeroTarget;
    std::vector<double> mUpLmt;
    std::vector<double> mDownLmt;
    std::vector<double> mCmdScale;
    std::vector<double> mSingleObs;
    std::vector<double> mSingleObsMimic;

    NLF2 mNlf2[50];

    int mLogData;

    static Ort::Env mOnnxEnv(ORT_LOGGING_LEVEL_WARNING, "policy");
    static void OnnxCtxDeinit(OnnxCtx *ctx)
    {
        if (ctx->input_tensor)
        {
            delete ctx->input_tensor;
            ctx->input_tensor = nullptr;
        }

        if (ctx->output_tensor)
        {
            delete ctx->output_tensor;
            ctx->output_tensor = nullptr;
        }

        if (ctx->memory_info)
        {
            delete ctx->memory_info;
            ctx->memory_info = nullptr;
        }

        if (ctx->session)
        {
            delete ctx->session;
            ctx->session = nullptr;
        }

        if (ctx->input_buffer)
        {
            free(ctx->input_buffer);
            ctx->input_buffer = nullptr;
        }

        if (ctx->output_buffer)
        {
            free(ctx->output_buffer);
            ctx->output_buffer = nullptr;
        }

        if (ctx->input_name)
        {
            free(ctx->input_name);
            ctx->input_name = nullptr;
        }

        if (ctx->output_name)
        {
            free(ctx->output_name);
            ctx->output_name = nullptr;
        }
    }
    static void OnnxCtxInit(OnnxCtx *ctx, const char *model_path)
    {
        OnnxCtxDeinit(ctx);

        ctx->env = &mOnnxEnv;

        Ort::SessionOptions opts;
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        opts.SetExecutionMode(ORT_SEQUENTIAL);
        opts.SetIntraOpNumThreads(1);

        ctx->session = new Ort::Session(*ctx->env, model_path, opts);

        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name_ptr = ctx->session->GetInputNameAllocated(0, allocator);
        ctx->input_name = strdup(input_name_ptr.get());
        auto output_name_ptr = ctx->session->GetOutputNameAllocated(0, allocator);
        ctx->output_name = strdup(output_name_ptr.get());

        // shapes
        auto shape = ctx->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        assert(shape.size() == 2);
        ctx->input_shape[0] = 1;
        ctx->input_shape[1] = shape[1];
        shape = ctx->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        assert(shape.size() == 2);
        ctx->output_shape[0] = 1;
        ctx->output_shape[1] = shape[1];

        // malloc buffers
        ctx->input_buffer = (float *)calloc(ctx->input_shape[1], sizeof(float));
        ctx->output_buffer = (float *)calloc(ctx->output_shape[1], sizeof(float));

        ctx->memory_info = new Ort::MemoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

        // create tensors ONCE
        ctx->input_tensor = new Ort::Value(
            Ort::Value::CreateTensor<float>(
                *ctx->memory_info,
                ctx->input_buffer,
                ctx->input_shape[1],
                ctx->input_shape,
                2));

        ctx->output_tensor = new Ort::Value(
            Ort::Value::CreateTensor<float>(
                *ctx->memory_info,
                ctx->output_buffer,
                ctx->output_shape[1],
                ctx->output_shape,
                2));
    }

    void KpiConfigInit(const char *configp_path)
    {
        mYamlNode = YAML::LoadFile(configp_path);
        mActionScale = mYamlNode["action_scale"].as<double>();
        mOmegaScale = mYamlNode["ang_vel_scale"].as<double>();
        mDofPosScale = mYamlNode["dof_pos_scale"].as<double>();
        mDofVelScale = mYamlNode["dof_vel_scale"].as<double>();
        mCmdScale = mYamlNode["cmd_scale"].as<std::vector<double>>();
        mNumAction = mYamlNode["num_actions"].as<int>();
        mNumSingleObs = mYamlNode["num_obs"].as<int>();
        mNumFrameStack = mYamlNode["num_frame_stack"].as<int>();
        mJointNames = mYamlNode["joint_names"].as<std::vector<std::string>>();
        assert(mJointNames.size() == (unsigned int)mNumAction);
        mKp = mYamlNode["kp"].as<std::vector<double>>();
        mKd = mYamlNode["kd"].as<std::vector<double>>();
        mKp_PR = mYamlNode["kp_pitch_roll"].as<std::vector<double>>();
        mKd_PR = mYamlNode["kd_pitch_roll"].as<std::vector<double>>();
        mDefaultPos = mYamlNode["default_angles"].as<std::vector<double>>();
        assert(mDefaultPos.size() == (unsigned int)mNumAction);
        mUpLmt = mYamlNode["up_limit"].as<std::vector<double>>();
        assert(mUpLmt.size() == (unsigned int)mNumAction);
        mDownLmt = mYamlNode["down_limit"].as<std::vector<double>>();
        assert(mDownLmt.size() == (unsigned int)mNumAction);
        mInitTarget = mYamlNode["init_target"].as<std::vector<double>>();
        assert(mInitTarget.size() == (unsigned int)mNumAction);
        mZeroTarget = mYamlNode["zero_target"].as<std::vector<double>>();
        assert(mZeroTarget.size() == (unsigned int)mNumAction);
        std::string policy_path = mYamlNode["policy_path"].as<std::string>();
        if (!policy_path.empty() && policy_path[0] != '/')
            policy_path = std::string("./config") + "/" + policy_path;
        OnnxCtxInit(&mLocoPolicy, policy_path.c_str());
        assert(mNumSingleObs * mNumFrameStack == mLocoPolicy.input_shape[1]);
        assert(mNumAction == mLocoPolicy.output_shape[1]);
        mAngleMapFlag = mYamlNode["angle_map"].as<int>();
        double v_max = mYamlNode["pln_v_max"].as<double>();
        double a_max = mYamlNode["pln_a_max"].as<double>();
        mTimeStep = mYamlNode["pln_Ts"].as<double>();
        mLogData = mYamlNode["log_data"].as<int>();

        // OpenHLM 风格 state 发布绑定地址(供独立录制进程 record_jaka_zmq 订阅)
        if (mYamlNode["state_zmq_bind"])
            mStateZmqBind = mYamlNode["state_zmq_bind"].as<std::string>();

        mNumSingleObsMimic = mYamlNode["mimic_num_obs"]
            ? mYamlNode["mimic_num_obs"].as<int>()
            : (5 * mNumAction + 9);

        if (mYamlNode["mimic_policy_path"])
        {
            std::string mimic_policy_path = mYamlNode["mimic_policy_path"].as<std::string>();
            if (!mimic_policy_path.empty() && mimic_policy_path[0] != '/')
                mimic_policy_path = std::string("./config") + "/" + mimic_policy_path;
            OnnxCtxInit(&mMimicPolicy, mimic_policy_path.c_str());

            assert(mNumSingleObsMimic > 0);
            if (!mYamlNode["mimic_num_frame_stack"])
            {
                mNumFrameStackMimic = mMimicPolicy.input_shape[1] / mNumSingleObsMimic;
                assert(mNumSingleObsMimic * mNumFrameStackMimic == mMimicPolicy.input_shape[1]);
            }
            else
            {
                mNumFrameStackMimic = mYamlNode["mimic_num_frame_stack"].as<int>();
            }
            assert(mNumAction == mMimicPolicy.output_shape[1]);
        }
        else
        {
            mNumFrameStackMimic = mYamlNode["mimic_num_frame_stack"]
                ? mYamlNode["mimic_num_frame_stack"].as<int>()
                : mNumFrameStack;
        }

        if (mYamlNode["mimic_command_path"])
        {
            std::string cmd_path = mYamlNode["mimic_command_path"].as<std::string>();
            if (!cmd_path.empty() && cmd_path[0] != '/')
                cmd_path = std::string("./config") + "/" + cmd_path;
            std::ifstream file(cmd_path);
            if (file.is_open())
            {
                std::string line;
                while (std::getline(file, line))
                {
                    if (line.empty())
                        continue;
                    std::istringstream ss(line);
                    std::string cell;
                    std::vector<float> row;
                    while (std::getline(ss, cell, ','))
                    {
                        row.push_back(std::stof(cell));
                    }

                    Eigen::Quaternionf quat(row[6], row[3], row[4], row[5]);
                    mRefMotion.root_ori.push_back(quat.normalized());

                    std::vector<float> pos(row.begin() + 7, row.begin() + 7 + mNumAction);
                    mRefMotion.dof_pos.push_back(pos);
                }
                mRefMotion.motion_frames = static_cast<int>(mRefMotion.dof_pos.size());

                mRefMotion.dof_vel.resize(mRefMotion.motion_frames);
                for (int f = 0; f < mRefMotion.motion_frames; f++)
                {
                    mRefMotion.dof_vel[f].resize(mNumAction, 0.0f);
                    if (f > 0 && mRefMotion.dof_pos[f].size() >= (unsigned int)mNumAction)
                    {
                        for (int i = 0; i < mNumAction; i++)
                        {
                            mRefMotion.dof_vel[f][i] = (mRefMotion.dof_pos[f][i] - mRefMotion.dof_pos[f - 1][i]) / static_cast<float>(mTimeStep);
                        }
                    }
                }

                if (mDebugFlag)
                    printf("[KpiConfig] loaded mimic command: %d frames\n", mRefMotion.motion_frames);
            }
        }

        /********************* jaka-mini mimic ZMQ config ************************/
        if (mYamlNode["jakaminimimic_zmq"])
        {
            mJakaMiniZmqEnabled = true;
            YAML::Node cfg = mYamlNode["jakaminimimic_zmq"];

            std::string zmq_policy_path = cfg["policy_path"] ?
                cfg["policy_path"].as<std::string>() : "";
            if (!zmq_policy_path.empty())
            {
                if (!zmq_policy_path.empty() && zmq_policy_path[0] != '/')
                    zmq_policy_path = std::string("./config") + "/" + zmq_policy_path;
                OnnxCtxInit(&mMimicJakaMiniZmqPolicy, zmq_policy_path.c_str());
                mMimicJakaMiniZmqPolicy.session->Run(
                    Ort::RunOptions(nullptr),
                    &mMimicJakaMiniZmqPolicy.input_name,
                    mMimicJakaMiniZmqPolicy.input_tensor,
                    1,
                    &mMimicJakaMiniZmqPolicy.output_name,
                    mMimicJakaMiniZmqPolicy.output_tensor,
                    1); // warm up
                assert(mNumAction == mMimicJakaMiniZmqPolicy.output_shape[1]);
            }

            mJakaMiniZmqConnect = cfg["zmq_connect"] ?
                cfg["zmq_connect"].as<std::string>() : mJakaMiniZmqConnect;
            mJakaMiniZmqHwm = cfg["zmq_hwm"] ?
                cfg["zmq_hwm"].as<int>() : mJakaMiniZmqHwm;
            mJakaMiniZmqDt = cfg["dt_s"] ?
                cfg["dt_s"].as<double>() : mJakaMiniZmqDt;
            mJakaMiniZmqTolerance = cfg["tolerance_s"] ?
                cfg["tolerance_s"].as<double>() : mJakaMiniZmqTolerance;

            mJakaMiniZmqFutureSteps = cfg["future_steps"] ?
                cfg["future_steps"].as<std::vector<int>>()
                : std::vector<int>{0, 1, 2, 3, 4};

            if (cfg["default_joint_pos"] && cfg["default_joint_pos"].size() > 0)
            {
                auto def_jpos = cfg["default_joint_pos"].as<std::vector<double>>();
                mJakaMiniZmqDefaultJointPos = Eigen::VectorXf(def_jpos.size());
                for (size_t i = 0; i < def_jpos.size(); ++i)
                    mJakaMiniZmqDefaultJointPos[i] = static_cast<float>(def_jpos[i]);
            }

            if (cfg["default_root_pos"] && cfg["default_root_pos"].size() >= 3)
            {
                auto def_rpos = cfg["default_root_pos"].as<std::vector<double>>();
                mJakaMiniZmqDefaultRootPos = Eigen::Vector3f(
                    static_cast<float>(def_rpos[0]),
                    static_cast<float>(def_rpos[1]),
                    static_cast<float>(def_rpos[2]));
            }

            if (cfg["default_root_quat"] && cfg["default_root_quat"].size() >= 4)
            {
                auto def_rquat = cfg["default_root_quat"].as<std::vector<double>>();
                mJakaMiniZmqDefaultRootQuat = Eigen::Quaternionf(
                    static_cast<float>(def_rquat[0]),
                    static_cast<float>(def_rquat[1]),
                    static_cast<float>(def_rquat[2]),
                    static_cast<float>(def_rquat[3]));
                mJakaMiniZmqDefaultRootQuat.normalize();
            }

            std::printf("JakaMini ZMQ mimic config loaded: zmq=%s, dt=%.3f, tol=%.3f\n",
                        mJakaMiniZmqConnect.c_str(), mJakaMiniZmqDt, mJakaMiniZmqTolerance);
        }

        /********************* jaka-mini mimic ZMQ fixwrist config ************************/
        if (mYamlNode["jakaminimimic_zmq_fixwrist"])
        {
            mJakaMiniZmqFixwristEnabled = true;
            YAML::Node cfg = mYamlNode["jakaminimimic_zmq_fixwrist"];

            std::string zmq_policy_path = cfg["policy_path"] ?
                cfg["policy_path"].as<std::string>() : "";
            if (!zmq_policy_path.empty())
            {
                if (!zmq_policy_path.empty() && zmq_policy_path[0] != '/')
                    zmq_policy_path = std::string("./config") + "/" + zmq_policy_path;
                OnnxCtxInit(&mMimicJakaMiniZmqFixwristPolicy, zmq_policy_path.c_str());
                mMimicJakaMiniZmqFixwristPolicy.session->Run(
                    Ort::RunOptions(nullptr),
                    &mMimicJakaMiniZmqFixwristPolicy.input_name,
                    mMimicJakaMiniZmqFixwristPolicy.input_tensor,
                    1,
                    &mMimicJakaMiniZmqFixwristPolicy.output_name,
                    mMimicJakaMiniZmqFixwristPolicy.output_tensor,
                    1); // warm up
                assert(23 == mMimicJakaMiniZmqFixwristPolicy.output_shape[1]);
            }

            mJakaMiniZmqFixwristConnect = cfg["zmq_connect"] ?
                cfg["zmq_connect"].as<std::string>() : mJakaMiniZmqFixwristConnect;
            mJakaMiniZmqFixwristHwm = cfg["zmq_hwm"] ?
                cfg["zmq_hwm"].as<int>() : mJakaMiniZmqFixwristHwm;
            mJakaMiniZmqFixwristDt = cfg["dt_s"] ?
                cfg["dt_s"].as<double>() : mJakaMiniZmqFixwristDt;
            mJakaMiniZmqFixwristTolerance = cfg["tolerance_s"] ?
                cfg["tolerance_s"].as<double>() : mJakaMiniZmqFixwristTolerance;

            mJakaMiniZmqFixwristFutureSteps = cfg["future_steps"] ?
                cfg["future_steps"].as<std::vector<int>>()
                : std::vector<int>{0, 1, 2, 3, 4};

            if (cfg["default_joint_pos"] && cfg["default_joint_pos"].size() > 0)
            {
                auto def_jpos = cfg["default_joint_pos"].as<std::vector<double>>();
                mJakaMiniZmqFixwristDefaultJointPos = Eigen::VectorXf(def_jpos.size());
                for (size_t i = 0; i < def_jpos.size(); ++i)
                    mJakaMiniZmqFixwristDefaultJointPos[i] = static_cast<float>(def_jpos[i]);
            }

            if (cfg["default_root_pos"] && cfg["default_root_pos"].size() >= 3)
            {
                auto def_rpos = cfg["default_root_pos"].as<std::vector<double>>();
                mJakaMiniZmqFixwristDefaultRootPos = Eigen::Vector3f(
                    static_cast<float>(def_rpos[0]),
                    static_cast<float>(def_rpos[1]),
                    static_cast<float>(def_rpos[2]));
            }

            if (cfg["default_root_quat"] && cfg["default_root_quat"].size() >= 4)
            {
                auto def_rquat = cfg["default_root_quat"].as<std::vector<double>>();
                mJakaMiniZmqFixwristDefaultRootQuat = Eigen::Quaternionf(
                    static_cast<float>(def_rquat[0]),
                    static_cast<float>(def_rquat[1]),
                    static_cast<float>(def_rquat[2]),
                    static_cast<float>(def_rquat[3]));
                mJakaMiniZmqFixwristDefaultRootQuat.normalize();
            }

            std::printf("JakaMini ZMQ fixwrist mimic config loaded: zmq=%s, dt=%.3f, tol=%.3f\n",
                        mJakaMiniZmqFixwristConnect.c_str(), mJakaMiniZmqFixwristDt, mJakaMiniZmqFixwristTolerance);
        }

        /********************* Khan SuperMimic real config **********************/
        if (mYamlNode["jakaminimimic_superzmq"])
        {
            const YAML::Node cfg = mYamlNode["jakaminimimic_superzmq"];
            // Real hardware gains/defaults always remain those read above from
            // YAML. No metadata override of mKp/mKd/mKp_PR/mKd_PR/mInitTarget.
            const auto require_vector = [](const std::vector<double> &values, size_t size,
                                           const char *field, bool nonnegative) {
                if (values.size() != size || !std::all_of(values.begin(), values.end(),
                    [nonnegative](double value) { return std::isfinite(value) && (!nonnegative || value >= 0); }))
                    throw std::runtime_error(std::string("Invalid real YAML ") + field);
            };
            if (mNumAction != 27 || !std::isfinite(mTimeStep) || std::abs(mTimeStep-.02) > 1.e-9)
                throw std::runtime_error("SuperMimic requires 27 joints and YAML pln_Ts=0.02");
            require_vector(mKp,27,"kp",true);
            require_vector(mKd,27,"kd",true);
            require_vector(mKp_PR,4,"kp_pitch_roll",true);
            require_vector(mKd_PR,4,"kd_pitch_roll",true);
            require_vector(mInitTarget,27,"init_target",false);
            require_vector(mUpLmt,27,"up_limit",false);
            require_vector(mDownLmt,27,"down_limit",false);
            for (int i=0;i<27;++i)
                if (mDownLmt[i]>mUpLmt[i]) throw std::runtime_error("Real YAML joint limits are reversed");

            mJakaMiniSuperZmqConnect=cfg["zmq_connect"].as<std::string>(mJakaMiniSuperZmqConnect);
            mJakaMiniSuperZmqHwm=cfg["zmq_hwm"].as<int>(1);
            mJakaMiniSuperZmqDt=cfg["dt_s"].as<double>(.02);
            mJakaMiniSuperZmqTolerance=cfg["tolerance_s"].as<double>(.04);
            mJakaMiniSuperZmqFutureSteps=cfg["future_steps"].as<std::vector<int>>(std::vector<int>{0,1,2,3,4});
            validate_motion_sampling(mJakaMiniSuperZmqDt,mJakaMiniSuperZmqTolerance,mJakaMiniSuperZmqFutureSteps);
            if (std::abs(mJakaMiniSuperZmqDt-.02)>1.e-9 ||
                mJakaMiniSuperZmqFutureSteps!=std::vector<int>({0,1,2,3,4}) || mJakaMiniSuperZmqHwm<1)
                throw std::runtime_error("SuperMimic requires dt_s=.02, future_steps=[0,1,2,3,4], positive HWM");
            if (cfg["reference_body_index"] && cfg["reference_body_index"].as<int>()!=0)
                throw std::runtime_error("SuperMimic reference is fixed to base_link index 0");
            mJakaMiniSuperZmqTiming=load_motion_timing(cfg);
            if (mJakaMiniSuperZmqTiming.stale_timeout_ns<=0 || mJakaMiniSuperZmqTiming.reset_gap_ns<=0 ||
                mJakaMiniSuperZmqTiming.discontinuity_ns<=0)
                throw std::runtime_error("SuperMimic clock timeouts must be positive");
            mJakaMiniSuperZmqRootHeightOffset=cfg["reference_height_offset"].as<double>(.05);
            if (!std::isfinite(mJakaMiniSuperZmqRootHeightOffset)||std::abs(mJakaMiniSuperZmqRootHeightOffset)>1)
                throw std::runtime_error("Invalid SuperMimic reference height offset");
            if (cfg["default_root_pos"])
            {
                const auto pos=cfg["default_root_pos"].as<std::vector<float>>();
                if(pos.size()!=3)throw std::runtime_error("SuperMimic default_root_pos must have 3 values");
                mJakaMiniSuperZmqDefaultRootPos=Eigen::Vector3f(pos[0],pos[1],pos[2]);
                if(!mJakaMiniSuperZmqDefaultRootPos.allFinite())throw std::runtime_error("Nonfinite default root position");
            }
            if (cfg["default_root_quat"])
            {
                const auto q=cfg["default_root_quat"].as<std::vector<float>>();
                if(q.size()!=4)throw std::runtime_error("SuperMimic default_root_quat must have 4 values");
                const Eigen::Quaternionf quat(q[0],q[1],q[2],q[3]);
                if(!quat.coeffs().allFinite()||quat.norm()<1.e-6f)throw std::runtime_error("Invalid default root quaternion");
                mJakaMiniSuperZmqDefaultRootQuat=quat.normalized();
            }
            std::string model=cfg["policy_path"].as<std::string>();
            if(model.empty())throw std::runtime_error("SuperMimic policy_path is empty");
            if(model.front()!='/')model="./config/"+model;
            OnnxCtxInit(&mMimicJakaMiniSuperZmqPolicy,model.c_str());
            mJakaMiniSuperDeployment=std::make_unique<SuperMimicDeployment>(
                *mMimicJakaMiniSuperZmqPolicy.session,cfg,mJointNames,mDefaultPos,mActionScale);
            mJakaMiniSuperZmqDefaultJointPos=mJakaMiniSuperDeployment->buffer_default_joint_pos();
            mMimicJakaMiniSuperZmqPolicy.session->Run(
                Ort::RunOptions(nullptr),&mMimicJakaMiniSuperZmqPolicy.input_name,
                mMimicJakaMiniSuperZmqPolicy.input_tensor,1,&mMimicJakaMiniSuperZmqPolicy.output_name,
                mMimicJakaMiniSuperZmqPolicy.output_tensor,1);
            mJakaMiniSuperZmqEnabled=true;
            std::printf("[SuperMimic real] 620 -> 27 actor loaded; joint parameters and hardware PD from YAML, no metadata override\n");
        }

        mPt.positions.resize(mNumAction);

        /*************************************************************/
        mLocoPolicy.session->Run(
            Ort::RunOptions(nullptr),
            &mLocoPolicy.input_name,
            mLocoPolicy.input_tensor,
            1,
            &mLocoPolicy.output_name,
            mLocoPolicy.output_tensor,
            1); // warm up

        mSingleObs.resize(mNumSingleObs);
        mSingleObsMimic.resize(mNumSingleObsMimic);

        int i;
        for (i = 0; i < mNumAction; i++)
            NLF2Init(&mNlf2[i], mTimeStep, a_max, v_max, 0.0, 0.0);

        if (mAngleMapFlag)
        {
            double a, b;
            PitchRoll2AB(mInitTarget[4], mInitTarget[5], &a, &b);
            mInitTarget[4] = a;
            mInitTarget[5] = b;
            PitchRoll2AB(mInitTarget[16], mInitTarget[17], &a, &b);
            mInitTarget[16] = -a;
            mInitTarget[17] = -b;
        }
        else
        {
            const auto lower=mYamlNode["pitch_roll_down_limit"].as<std::vector<double>>(
                std::vector<double>{-.52,-.35,-.52,-.35});
            const auto upper=mYamlNode["pitch_roll_up_limit"].as<std::vector<double>>(
                std::vector<double>{.61,.35,.61,.35});
            if(lower.size()!=4||upper.size()!=4)
                throw std::runtime_error("Physical PR limits must contain four values");
            const int indices[4]={4,5,16,17};
            for(int i=0;i<4;++i)
            {
                if(!std::isfinite(lower[i])||!std::isfinite(upper[i])||lower[i]>upper[i])
                    throw std::runtime_error("Invalid physical PR YAML limits");
                mDownLmt[indices[i]]=lower[i];
                mUpLmt[indices[i]]=upper[i];
            }
        }
    }

    void KpiConfigDeinit()
    {
        mJakaMiniSuperDeployment.reset();
        OnnxCtxDeinit(&mMimicJakaMiniSuperZmqPolicy);
        mJakaMiniSuperZmqEnabled=false;
        OnnxCtxDeinit(&mLocoPolicy);
        if (mMimicPolicy.session)
        {
            OnnxCtxDeinit(&mMimicPolicy);
        }
        if (mMimicJakaMiniZmqPolicy.session)
        {
            OnnxCtxDeinit(&mMimicJakaMiniZmqPolicy);
        }
        if (mMimicJakaMiniZmqFixwristPolicy.session)
        {
            OnnxCtxDeinit(&mMimicJakaMiniZmqFixwristPolicy);
        }
    }
}
