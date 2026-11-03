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
#include "RLController.hpp"

namespace RLController
{
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
            mUpLmt[4] = 0.61;
            mUpLmt[5] = 0.35;
            mUpLmt[16] = 0.61;
            mUpLmt[17] = 0.35;

            mDownLmt[4] = -0.52;
            mDownLmt[5] = -0.35;
            mDownLmt[16] = -0.52;
            mDownLmt[17] = -0.35;
        }
    }

    void KpiConfigDeinit()
    {
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
