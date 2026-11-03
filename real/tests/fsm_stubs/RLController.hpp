#pragma once
// Test-only I/O and inference substitutes. The FSM and ankle safety .cpp files
// themselves are compiled unchanged. This header cannot enter the app target.
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "RealtimeMotionBuffer.hpp"
#include "ParKine.hpp"

namespace Ort {struct RunOptions {explicit RunOptions(std::nullptr_t){}};}

namespace legged_waist_controller
{
    class RobotIntf
    {
    public:
        enum class ControlSource {JOY,OTHER};
        struct Joy
        {
            bool buttons_A=false,buttons_B=false,buttons_X=false,buttons_Y=false;
            bool buttons_LB=false,buttons_RB=false,buttons_START=false,buttons_BACK=false;
            float axes_DY=0;
        };
        struct JoyRobotState {Joy joy{};bool joy_online=true;};
        struct ImuData {double qw=1,qx=0,qy=0,qz=0,wx=0,wy=0,wz=0,ax=0,ay=0,az=0;};
        struct JointStatus {double pos_cmd=0,pos_fdb=0,vel_fdb=0;bool is_enable=true,is_fault=false;};
        ControlSource source=ControlSource::JOY;
        JoyRobotState joystick{};
        ImuData imu{};
        std::array<JointStatus,27> joints{};
        std::array<int,27> joint_result{};
        bool joy_ok=true,imu_ok=true;
        int enable_on=0,enable_off=0,pd_off=0,trajectory_attempts=0,trajectory_accepted=0;
        int trajectory_result=0,control_state=-1;
        std::vector<double> last_target;
        ControlSource get_control_source()const{return source;}
        bool get_joy_robot_state(JoyRobotState &out)const{out=joystick;return joy_ok;}
        bool get_imu(ImuData &out)const{out=imu;return imu_ok;}
        int get_joint_status(const std::string &name,JointStatus &out)const
        {const size_t i=std::stoul(name.substr(1));out=joints.at(i);return joint_result.at(i);}
        int do_enable(bool on){if(on)++enable_on;else ++enable_off;return 0;}
        int enable_pd(bool on){if(!on)++pd_off;return 0;}
        int set_cur_control_stat(int state){control_state=state;return 0;}
        template<class Point> int set_joint_trajectory(const std::vector<std::string>&,const Point &point)
        {++trajectory_attempts;if(trajectory_result>=0){++trajectory_accepted;last_target=point.positions;}return trajectory_result;}
    };
}

namespace RLController
{
    enum FSMSTATE {FSM_INIT=0,FSM_FAULT,FSM_IDLE,FSM_LOCO,FSM_MIMIC=10,
        FSM_JAKAMINIMIMIC_ZMQ,FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST,FSM_JAKAMINIMIMIC_SUPERZMQ};
    enum ERRCODE {ERR_NONE=0,ERR_INIT,ERR_STATE_TRANSITION,ERR_JOINT,ERR_ABORT};
    using fsm_func_ptr=void(*)();
    struct FakeSession
    {
        std::array<float,27> action{};
        float *output=nullptr;
        bool fail=false;
        int calls=0;
        template<class... Args>void Run(Args&&...)
        {++calls;if(fail)throw std::runtime_error("injected inference failure");std::copy(action.begin(),action.end(),output);}
    };
    struct OnnxCtx
    {
        FakeSession *session=nullptr;
        char *input_name=nullptr,*output_name=nullptr;
        int64_t input_shape[2]{1,620},output_shape[2]{1,27};
        float *input_buffer=nullptr,*output_buffer=nullptr;
        void *input_tensor=nullptr,*output_tensor=nullptr;
    };
    struct SuperMimicDeployment
    {
        std::array<int,27> policy_to_deploy{},policy_from_buffer{};
        std::array<float,27> default_joint_pos{},action_scale{};
        float angular_velocity_scale=.5f,joint_velocity_scale=.2f;
    };
    struct Point {std::vector<double> positions;rclcpp::Duration time_from_start;};
    extern std::unique_ptr<legged_waist_controller::RobotIntf> mRobot;
    extern Point mPt;
    extern bool mDataReadyFlag,mDebugFlag,mJakaMiniSuperZmqEnabled,mJakaMiniZmqEnabled,mJakaMiniZmqFixwristEnabled;
    extern int mNumAction,mEnabledJointCount,mFaultJointCount,mAngleMapFlag;
    extern FSMSTATE mFSMState;
    extern ERRCODE mErrCode;
    extern double mTimeStep;
    extern std::vector<std::string> mJointNames;
    extern std::vector<double> mDefaultPos,mInitTarget,mZeroTarget,mDownLmt,mUpLmt;
    extern NLF2 mNlf2[50];
    extern OnnxCtx mMimicJakaMiniSuperZmqPolicy;
    extern std::unique_ptr<SuperMimicDeployment> mJakaMiniSuperDeployment;
    extern std::string mJakaMiniSuperZmqConnect,mStateZmqBind;
    extern int mJakaMiniSuperZmqHwm;
    extern double mJakaMiniSuperZmqDt,mJakaMiniSuperZmqTolerance;
    extern std::vector<int> mJakaMiniSuperZmqFutureSteps;
    extern Eigen::VectorXf mJakaMiniSuperZmqDefaultJointPos;
    extern Eigen::Vector3f mJakaMiniSuperZmqDefaultRootPos;
    extern Eigen::Quaternionf mJakaMiniSuperZmqDefaultRootQuat;
    extern fsm_func_ptr StateJudge,StateRun;
    extern FSMSTATE ReqState;
    void FSMUpdate();
    void FSMIdleEnter(FSMSTATE);
    void FSMFaultEnter(FSMSTATE);
    void FSMLocoEnter(FSMSTATE);
    void FSMMimicEnter(FSMSTATE);
    void FSMMimicJakaMiniZmqEnter(FSMSTATE);
    void FSMMimicJakaMiniZmqFixwristEnter(FSMSTATE);
    void FSMMimicJakaMiniSuperZmqEnter(FSMSTATE);
    void update_joint_status(double*,double*,double*);
    Eigen::Quaternionf yawQuaternion(const Eigen::Quaternionf&);
    void imu_process(double*,double*,double*);
}
