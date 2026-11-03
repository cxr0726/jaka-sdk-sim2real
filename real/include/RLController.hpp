#ifndef __JK_RLCONTROLLER_H__
#define __JK_RLCONTROLLER_H__

#include <string>
#include <vector>
#include "onnxruntime_cxx_api.h"
#include "ParKine.hpp"
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include "robot_intf.hpp"
#include "RealtimeMotionBuffer.hpp"
#include "SuperRealtimeMotionBuffer.hpp"
#include "SuperMimicDeployment.hpp"

namespace RLController
{
    /*********************** FSM *********************/
    typedef enum
    {
        FSM_INIT = 0, // no feedback
        FSM_FAULT,    // servo off
        FSM_IDLE,     // has feedback, servo on/off, w/o joint cmd
        FSM_LOCO,     // servo on, with joint cmd
        FSM_MIMIC = 10, // mimic reference motion via ONNX policy
        FSM_JAKAMINIMIMIC_ZMQ, // servo on, jaka-mini mimic with ZMQ real-time motion
        FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST, // servo on, jaka-mini mimic 23-DoF with ZMQ real-time motion
        FSM_JAKAMINIMIMIC_SUPERZMQ, // Khan SuperMimic, hardware gains remain from YAML
    } FSMSTATE;

    typedef enum
    {
        ERR_NONE = 0,
        ERR_INIT,
        ERR_STATE_TRANSITION,
        ERR_JOINT,
        ERR_ABORT,
    } ERRCODE;

    typedef void (*fsm_func_ptr)();

    extern void FSMUpdate();

    extern void FSMInitEnter(FSMSTATE from);
    extern void FSMFaultEnter(FSMSTATE from);
    extern void FSMIdleEnter(FSMSTATE from);
    extern void FSMLocoEnter(FSMSTATE from);
    extern void FSMMimicEnter(FSMSTATE from);
    extern void FSMMimicJakaMiniZmqEnter(FSMSTATE from);
    extern void FSMMimicJakaMiniZmqFixwristEnter(FSMSTATE from);
    extern void FSMMimicJakaMiniSuperZmqEnter(FSMSTATE from);

    struct RefMotion
    {
        int motion_frames = 0;
        std::vector<std::vector<float>> dof_pos;
        std::vector<std::vector<float>> dof_vel;
        std::vector<Eigen::Quaternionf> root_ori;
    };

    struct OnnxCtx // structure definition
    {
        Ort::Env *env;
        Ort::Session *session;

        char *input_name;
        char *output_name;

        int64_t input_shape[2];
        int64_t output_shape[2];

        float *input_buffer;
        float *output_buffer;

        Ort::MemoryInfo *memory_info;
        Ort::Value *input_tensor;
        Ort::Value *output_tensor;
    }; // for preloaded policies


    /******************** global vars declaration ******************/
    // Process-wide owner of the C++ bridge to the controller's ROS interfaces.
    extern std::unique_ptr<legged_waist_controller::RobotIntf> mRobot;
    extern trajectory_msgs::msg::JointTrajectoryPoint mPt;

    extern bool mRLThreadRunFlag;
    extern bool mDataReadyFlag;
    extern int mEnabledJointCount;
    extern int mFaultJointCount;

    extern FSMSTATE mFSMState;
    extern ERRCODE mErrCode;
    extern bool mDebugFlag;

    extern std::vector<double> mKp;
    extern std::vector<double> mKd;

    extern std::vector<double> mKp_PR;
    extern std::vector<double> mKd_PR;

    extern std::vector<std::string> mJointNames;

    extern int mAngleMapFlag;

    // yaml configs
    extern YAML::Node mYamlNode;

    extern OnnxCtx mLocoPolicy;
    extern int mNumSingleObs;
    extern int mNumFrameStack;
    extern int mNumAction;
    extern double mTimeStep;

    extern OnnxCtx mMimicPolicy;
    extern int mNumSingleObsMimic;
    extern int mNumFrameStackMimic;

    extern RefMotion mRefMotion;

    // jaka-mini mimic ZMQ settings
    extern OnnxCtx mMimicJakaMiniZmqPolicy;
    extern bool mJakaMiniZmqEnabled;
    extern std::string mJakaMiniZmqConnect;
    extern int mJakaMiniZmqHwm;
    extern double mJakaMiniZmqDt;
    extern double mJakaMiniZmqTolerance;
    extern std::vector<int> mJakaMiniZmqFutureSteps;
    extern Eigen::VectorXf mJakaMiniZmqDefaultJointPos;
    extern Eigen::Vector3f mJakaMiniZmqDefaultRootPos;
    extern Eigen::Quaternionf mJakaMiniZmqDefaultRootQuat;

    // jaka-mini mimic ZMQ fixwrist settings (23-DoF)
    extern OnnxCtx mMimicJakaMiniZmqFixwristPolicy;
    extern bool mJakaMiniZmqFixwristEnabled;
    extern std::string mJakaMiniZmqFixwristConnect;
    extern int mJakaMiniZmqFixwristHwm;
    extern double mJakaMiniZmqFixwristDt;
    extern double mJakaMiniZmqFixwristTolerance;
    extern std::vector<int> mJakaMiniZmqFixwristFutureSteps;
    extern Eigen::VectorXf mJakaMiniZmqFixwristDefaultJointPos;
    extern Eigen::Vector3f mJakaMiniZmqFixwristDefaultRootPos;
    extern Eigen::Quaternionf mJakaMiniZmqFixwristDefaultRootQuat;
    // SuperMimic: independent source-clock buffer and YAML-owned joint parameters.
    extern OnnxCtx mMimicJakaMiniSuperZmqPolicy;
    extern bool mJakaMiniSuperZmqEnabled;
    extern std::string mJakaMiniSuperZmqConnect;
    extern int mJakaMiniSuperZmqHwm;
    extern double mJakaMiniSuperZmqDt;
    extern double mJakaMiniSuperZmqTolerance;
    extern std::vector<int> mJakaMiniSuperZmqFutureSteps;
    extern Eigen::VectorXf mJakaMiniSuperZmqDefaultJointPos;
    extern Eigen::Vector3f mJakaMiniSuperZmqDefaultRootPos;
    extern Eigen::Quaternionf mJakaMiniSuperZmqDefaultRootQuat;
    extern MotionTimingConfig mJakaMiniSuperZmqTiming;
    extern double mJakaMiniSuperZmqRootHeightOffset;
    extern std::unique_ptr<SuperMimicDeployment> mJakaMiniSuperDeployment;

    // OpenHLM 风格 state 发布(ZMQ PUB, JAKA 关节序)
    extern std::string mStateZmqBind;
    extern double mActionScale;
    extern double mDofPosScale;
    extern double mDofVelScale;
    extern double mOmegaScale;

    extern std::vector<double> mDefaultPos;
    extern std::vector<double> mInitTarget;
    extern std::vector<double> mZeroTarget;
    extern std::vector<double> mUpLmt;
    extern std::vector<double> mDownLmt;
    extern std::vector<double> mCmdScale;
    extern std::vector<double> mSingleObs;
    extern std::vector<double> mSingleObsMimic;

    extern NLF2 mNlf2[50];

    extern int mLogData;


    void KpiConfigInit(const char *configp_path);
    void KpiConfigDeinit();

    /**************** Util functions *****************/
    Eigen::Quaternionf yawQuaternion(const Eigen::Quaternionf &q);
    void q2R(const double q[4], double R[3][3]);
    void imu_process(double acc[3], double omega[3], double quat[4]);
    void bind_nlf2_state();
    void update_joint_status(double *q_cmd, double *q, double *qd);
}

#endif
