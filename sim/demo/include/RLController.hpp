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
#include "SuperMimicDeployment.hpp"

namespace RLController
{
    /*********************** FSM *********************/
    typedef enum
    {
        FSM_INIT = 0, // no feedback
        FSM_FAULT,    // preset position, damping mode or servo off
        FSM_IDLE = 7,      // has feedback, servo on/off, w/o joint cmd
        FSM_LOCO,          // servo on, with joint cmd
        FSM_MIMIC = 10,    // mimic reference motion via ONNX policy
        FSM_JAKAMINIMIMIC_ZMQ, // servo on, jaka-mini mimic with ZMQ real-time motion
        FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST, // servo on, jaka-mini mimic 23-DoF with ZMQ real-time motion
        FSM_JAKAMINIMIMIC_SUPERZMQ, // metadata-driven SuperMimic teleoperation
    } FSMSTATE;

    typedef enum
    {
        MOT_TYPE_NONE = 0, // no request

        // basic operations
        MOT_TYPE_ENABLE = 901,
        MOT_TYPE_DISABLE = 902,
        MOT_TYPE_RESET = 903,
        MOT_TYPE_GO_HOME = 904, // need set target/home first
        MOT_TYPE_GO_LOCO_INITPOS = 905,
        MOT_TYPE_ABORT = 999, // goto FAULT and disable robot

        // FSM_JUMPS
        MOT_TYPE_SWITCH_TO_LOCO = FSM_LOCO,
        MOT_TYPE_SWITCH_TO_MIMIC = FSM_MIMIC,

    } MOT_TYPE_CODE;

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

    extern void FSMLocoPrimeOfflineFromCurrent();
    extern bool FSMLocoInferOfflineAction(std::vector<double> &action);

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
    extern std::unique_ptr<legged_waist_controller::RobotIntf> mRobot;
    extern trajectory_msgs::msg::JointTrajectoryPoint mPt;
    extern legged_waist_controller::RobotIntf::MoveTypeMap mMotTypeMap;

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

    extern int mEnableRPY;
    extern int mEnableOmega;
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

    extern bool mNoTargetEnabled;

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
    extern MotionTimingConfig mJakaMiniZmqTiming;
    extern int mJakaMiniZmqReferenceBodyIndex;

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
    extern MotionTimingConfig mJakaMiniZmqFixwristTiming;
    extern int mJakaMiniZmqFixwristReferenceBodyIndex;

    // SuperMimic has an independent model, joint mapping and reference configuration.
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
    extern bool mJakaMiniSuperZmqClipTargetPositions;
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
    extern std::vector<double> mNegTarget;
    extern std::vector<double> mPosTarget;
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
    // Original ParKine conversion shared by the FSMs and the SDK PD ABI.
    // A negative forward-solve result zeros that ankle's PR feedback, as in
    // the legacy ZMQ implementation; dimensions still receive normal checks.
    bool decode_parallel_ankles(double *q, double *qd, const char *context);
    bool decode_parallel_ankle_commands(double *q_cmd, const char *context);
    bool encode_parallel_ankle_targets(std::vector<double> &targets, const char *context);
    bool latch_ankle_fault(const char *context, const char *detail);
    void reset_parallel_ankle_observers();
}

#endif
