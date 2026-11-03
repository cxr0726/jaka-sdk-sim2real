// Exercise the unchanged production FSM code with test-only I/O substitutes.
// No hardware RobotIntf, ROS node, ZMQ socket or real actor session is created.
#include "RLController.hpp"
#include "SuperMimicSafety.hpp"
#include "StatePublisher.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

// GNU link wrapping preserves the actual legacy ParKine implementation while
// allowing a deterministic negative-return injection at its public C ABI.
extern "C" int __real_AB2PitchRoll(double,double,double,double,double*,double*,double*,double*,double[2][2]);
namespace legacy_ankle_test
{
    struct Call { double a,b,da,db; };
    std::vector<Call> calls;
    int fail_call = -1;
}
extern "C" int __wrap_AB2PitchRoll(double a,double b,double da,double db,
    double *p,double *r,double *dp,double *dr,double J[2][2])
{
    const int index=static_cast<int>(legacy_ankle_test::calls.size());
    legacy_ankle_test::calls.push_back({a,b,da,db});
    if(index==legacy_ankle_test::fail_call)
    { *p=9.;*r=8.;*dp=7.;*dr=6.;return -1; }
    return __real_AB2PitchRoll(a,b,da,db,p,r,dp,dr,J);
}

#include "../src/FSM.cpp"
#include "../src/FSMMimicJakaMiniSuperZmq.cpp"
#include "../src/FSMIdle.cpp"

namespace RLController
{
    std::unique_ptr<legged_waist_controller::RobotIntf> mRobot;
    Point mPt;
    bool mDataReadyFlag=true,mDebugFlag=false,mJakaMiniSuperZmqEnabled=true;
    bool mJakaMiniZmqEnabled=true,mJakaMiniZmqFixwristEnabled=true;
    int mNumAction=27,mEnabledJointCount=27,mFaultJointCount=0,mAngleMapFlag=0;
    FSMSTATE mFSMState=FSM_INIT;
    ERRCODE mErrCode=ERR_NONE;
    double mTimeStep=.02;
    std::vector<std::string> mJointNames;
    std::vector<double> mDefaultPos,mInitTarget,mZeroTarget,mDownLmt,mUpLmt;
    NLF2 mNlf2[50]{};
    OnnxCtx mMimicJakaMiniSuperZmqPolicy;
    std::unique_ptr<SuperMimicDeployment> mJakaMiniSuperDeployment;
    std::string mJakaMiniSuperZmqConnect,mStateZmqBind;
    int mJakaMiniSuperZmqHwm=1;
    double mJakaMiniSuperZmqDt=.02,mJakaMiniSuperZmqTolerance=.04;
    std::vector<int> mJakaMiniSuperZmqFutureSteps={0,1,2,3,4};
    Eigen::VectorXf mJakaMiniSuperZmqDefaultJointPos=Eigen::VectorXf::Zero(27);
    Eigen::Vector3f mJakaMiniSuperZmqDefaultRootPos{0,0,.65f};
    Eigen::Quaternionf mJakaMiniSuperZmqDefaultRootQuat=Eigen::Quaternionf::Identity();

    void no_op(){}
    void FSMFaultEnter(FSMSTATE)
    {
        mFSMState=ReqState=FSM_FAULT;StateJudge=StateRun=&no_op;
        if(mRobot)mRobot->do_enable(false);
    }
    void FSMLocoEnter(FSMSTATE){mFSMState=ReqState=FSM_LOCO;StateJudge=StateRun=&no_op;}
    void FSMMimicEnter(FSMSTATE){mFSMState=ReqState=FSM_MIMIC;StateJudge=StateRun=&no_op;}
    void FSMMimicJakaMiniZmqEnter(FSMSTATE){mFSMState=ReqState=FSM_JAKAMINIMIMIC_ZMQ;StateJudge=StateRun=&no_op;}
    void FSMMimicJakaMiniZmqFixwristEnter(FSMSTATE){mFSMState=ReqState=FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST;StateJudge=StateRun=&no_op;}
    void update_joint_status(double *cmd,double *pos,double *vel)
    {
        mEnabledJointCount=mFaultJointCount=0;
        for(int i=0;i<27;++i)
        {
            const auto &state=mRobot->joints[i];cmd[i]=state.pos_cmd;pos[i]=state.pos_fdb;vel[i]=state.vel_fdb;
            mEnabledJointCount+=state.is_enable;mFaultJointCount+=state.is_fault;
        }
    }
    // Fault/transition tests use identity installation alignment. Numerical
    // IMU, policy and joint-order parity is tested separately against sim.
    void imu_process(double*,double*,double*){}
    Eigen::Quaternionf yawQuaternion(const Eigen::Quaternionf &q)
    {
        const float yaw=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
        return Eigen::Quaternionf(Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ()));
    }
}

namespace
{
    using namespace RLController;
    using Robot=legged_waist_controller::RobotIntf;
    FakeSession actor;
    std::array<float,620> input{};
    std::array<float,27> output{};
    int cases=0;

    void require(bool value,const char *why){if(!value)throw std::runtime_error(why);}
    void noop_nlf(void*,double,double){}

    void fixture()
    {
        zmq_motion_buffer.reset();zmq_state_pub.reset();zmq_policy_diagnostic.close();
        mRobot=std::make_unique<Robot>();mErrCode=ERR_NONE;
        mDataReadyFlag=true;mEnabledJointCount=27;mFaultJointCount=0;mAngleMapFlag=0;
        mJakaMiniSuperZmqEnabled=mJakaMiniZmqEnabled=mJakaMiniZmqFixwristEnabled=true;
        mJointNames.clear();for(int i=0;i<27;++i)mJointNames.push_back("j"+std::to_string(i));
        mDefaultPos.assign(27,0);mInitTarget.assign(27,0);mZeroTarget.assign(27,0);
        mDownLmt.assign(27,-1);mUpLmt.assign(27,1);mPt.positions.assign(27,0);
        for(auto &n:mNlf2)n.step=&noop_nlf;
        mJakaMiniSuperDeployment=std::make_unique<SuperMimicDeployment>();
        for(int i=0;i<27;++i)
        {mJakaMiniSuperDeployment->policy_to_deploy[i]=i;mJakaMiniSuperDeployment->policy_from_buffer[i]=i;}
        mJakaMiniSuperDeployment->action_scale.fill(.5f);
        actor=FakeSession{};input.fill(0);output.fill(0);actor.output=output.data();
        mMimicJakaMiniSuperZmqPolicy=OnnxCtx{};mMimicJakaMiniSuperZmqPolicy.session=&actor;
        mMimicJakaMiniSuperZmqPolicy.input_buffer=input.data();mMimicJakaMiniSuperZmqPolicy.output_buffer=output.data();
        RealtimeMotionBuffer::fail_construct=RealtimeMotionBuffer::fail_query=false;
        RealtimeMotionBuffer::next_ready=false;
        RealtimeMotionBuffer::ready_calls=RealtimeMotionBuffer::query_calls=0;
        RealtimeMotionBuffer::next={};
        legacy_ankle_test::calls.clear();legacy_ankle_test::fail_call=-1;
        StatePublisher::fail_construct=false;StatePublisher::calls=0;
        FSMIdleEnter(FSM_INIT);
    }

    void chord(bool lb,bool b,bool a=false)
    {mRobot->joystick.joy={};mRobot->joystick.joy.buttons_LB=lb;mRobot->joystick.joy.buttons_B=b;mRobot->joystick.joy.buttons_A=a;}
    void enter()
    {
        chord(false,false);fsm_idle_judge();chord(true,true);fsm_idle_judge();
        require(mFSMState==FSM_JAKAMINIMIMIC_SUPERZMQ && mErrCode==ERR_NONE,"healthy LB+B did not enter actual SuperMimic");
    }
    void no_command_fault(const std::function<void()> &change,const char *why)
    {
        fixture();enter();change();fsm_mimic_jaka_mini_superzmq_run();
        require(mErrCode!=ERR_NONE,why);require(mRobot->pd_off>0&&mRobot->enable_off>0,"fault omitted PD/servo-off request");
        require(mRobot->trajectory_attempts==0,"invalid feedback/reference/output issued trajectory");++cases;
    }

    void transition_checks()
    {
        fixture();chord(true,true);fsm_idle_judge();
        require(mFSMState==FSM_IDLE,"held old MIMIC return chord entered Super without release");
        chord(false,false);fsm_idle_judge();chord(true,true);fsm_idle_judge();
        require(mFSMState==FSM_JAKAMINIMIMIC_SUPERZMQ,"fresh LB+B not accepted");++cases;
        fsm_mimic_jaka_mini_superzmq_judge();
        require(mFSMState==FSM_JAKAMINIMIMIC_SUPERZMQ,"held entry chord immediately exited Super");
        chord(false,false);fsm_mimic_jaka_mini_superzmq_judge();
        chord(true,true);fsm_mimic_jaka_mini_superzmq_judge();
        require(mFSMState==FSM_IDLE,"fresh LB+B did not return to actual IDLE");++cases;

        // Physical enable feedback is still true while IDLE's local flag was
        // reset on the transition. A must still request servo-off.
        require(!enable_robot,"actual IDLE entry did not reset local enable flag");
        chord(false,false,true);FSMUpdate();
        require(mRobot->enable_off>0&&mRobot->trajectory_attempts==0,"A ignored after Super -> IDLE");++cases;

        for(int mode=0;mode<6;++mode)
        {
            fixture();chord(false,false);fsm_idle_judge();
            auto &j=mRobot->joystick.joy;j.buttons_A=true;j.buttons_LB=true;
            if(mode==0)j.buttons_BACK=true;
            if(mode==1)j.buttons_B=true;
            if(mode==2)j.buttons_START=true;
            if(mode==3)j.buttons_Y=true;
            if(mode==4)j.buttons_X=true;
            if(mode==5){j.buttons_BACK=true;j.buttons_START=true;j.buttons_B=true;}
            FSMUpdate();
            require(mFSMState==FSM_IDLE||mFSMState==FSM_FAULT,"A+mode chord entered a policy");
            require(mRobot->enable_on==0&&mRobot->enable_off>0,"A+BACK emitted enable pulse or missed disable");
            require(mRobot->trajectory_attempts==0,"A+chord emitted target");++cases;
        }
        for(int invalid=0;invalid<3;++invalid)
        {
            fixture();chord(false,false);fsm_idle_judge();
            if(invalid==0)mEnabledJointCount=26;
            if(invalid==1)mFaultJointCount=1;
            if(invalid==2)mDataReadyFlag=false;
            chord(true,true);fsm_idle_judge();
            require(mFSMState!=FSM_JAKAMINIMIMIC_SUPERZMQ,"unready IDLE entered Super");++cases;
        }
        // Verify the existing old-mode key dispatch remains unchanged.
        fixture();mRobot->joystick.joy.buttons_LB=true;mRobot->joystick.joy.buttons_START=true;fsm_idle_judge();
        require(mFSMState==FSM_JAKAMINIMIMIC_ZMQ,"old LB+START mapping changed");++cases;
        fixture();mRobot->joystick.joy.buttons_LB=true;mRobot->joystick.joy.buttons_BACK=true;fsm_idle_judge();
        require(mFSMState==FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST,"old LB+BACK mapping changed");++cases;
    }

    void fault_checks()
    {
        no_command_fault([]{mRobot->joint_result[7]=-1;},"missing joint not rejected");
        no_command_fault([]{mRobot->joints[8].pos_fdb=std::numeric_limits<double>::quiet_NaN();},"NaN joint not rejected");
        no_command_fault([]{mRobot->joints[9].vel_fdb=std::numeric_limits<double>::infinity();},"Inf velocity not rejected");
        no_command_fault([]{mRobot->joints[16].is_enable=false;},"disabled joint not rejected");
        no_command_fault([]{mRobot->joints[19].is_fault=true;},"joint error not rejected");
        no_command_fault([]{mDataReadyFlag=false;},"unready feedback not rejected");
        no_command_fault([]{mRobot->imu_ok=false;},"missing IMU not rejected");
        no_command_fault([]{mRobot->imu.qw=0;},"zero quaternion not rejected");
        no_command_fault([]{mRobot->imu.wx=std::numeric_limits<double>::quiet_NaN();},"NaN IMU not rejected");
        no_command_fault([]{RealtimeMotionBuffer::fail_query=true;},"motion-window exception not rejected");
        no_command_fault([]{RealtimeMotionBuffer::next[2].dof_pos[6]=std::numeric_limits<float>::quiet_NaN();},"NaN observation not rejected");
        no_command_fault([]{actor.fail=true;},"inference exception not rejected");
        no_command_fault([]{actor.action[12]=std::numeric_limits<float>::quiet_NaN();},"NaN action not rejected");
        no_command_fault([]{mAngleMapFlag=1;},"changed angle_map still issued a target");

        fixture();enter();mErrCode=ERR_JOINT;fsm_mimic_jaka_mini_superzmq_run();
        require(actor.calls==0&&mRobot->trajectory_attempts==0,"prelatched error still ran actor or target publication");++cases;
        fixture();enter();mRobot->trajectory_result=-1;fsm_mimic_jaka_mini_superzmq_run();
        require(mErrCode!=ERR_NONE&&mRobot->pd_off>0&&mRobot->enable_off>0,"trajectory rejection omitted stop");
        require(mRobot->trajectory_attempts==1&&mRobot->trajectory_accepted==0,"rejected trajectory counted accepted");++cases;

        for(int invalid=0;invalid<3;++invalid)
        {
            fixture();enter();
            if(invalid==0)mRobot->source=Robot::ControlSource::OTHER;
            if(invalid==1)mRobot->joy_ok=false;
            if(invalid==2)mRobot->joystick.joy_online=false;
            FSMUpdate();require(mFSMState==FSM_FAULT&&mRobot->enable_off>0,"lost JOY did not fault");
            require(mRobot->trajectory_attempts==0,"lost JOY still issued trajectory");++cases;
        }
        // Normal teleoperation ends by pressing A directly in SuperMimic. Test
        // that exact path, plus every chord that could compete with A.
        for(int mode=0;mode<7;++mode)
        {
            fixture();enter();chord(false,false,true);
            auto &j=mRobot->joystick.joy;
            if(mode>0)j.buttons_LB=true;
            if(mode==1)j.buttons_B=true;
            if(mode==2)j.buttons_START=true;
            if(mode==3)j.buttons_BACK=true;
            if(mode==4)j.buttons_Y=true;
            if(mode==5)j.buttons_X=true;
            if(mode==6)j.buttons_RB=true;
            FSMUpdate();
            require(mFSMState==FSM_FAULT&&mRobot->enable_off>0,"direct Super A or A+chord failed to fault/disable");
            require(mRobot->trajectory_attempts==0&&actor.calls==0,"Super A allowed policy/trajectory in same FSMUpdate");
            ++cases;
        }

        // Entry failure and unsupported angle-map must stop in the SAME
        // FSMUpdate, including its StateRun call after StateJudge transitions.
        for(int invalid=0;invalid<5;++invalid)
        {
            fixture();chord(false,false);fsm_idle_judge();
            if(invalid==0)mAngleMapFlag=1;
            if(invalid==1)mMimicJakaMiniSuperZmqPolicy.input_shape[1]=619;
            if(invalid==2)mRobot->imu_ok=false;
            if(invalid==3)RealtimeMotionBuffer::fail_construct=true;
            if(invalid==4)StatePublisher::fail_construct=true;
            chord(true,true);FSMUpdate();
            require(mErrCode!=ERR_NONE&&mRobot->enable_off>0,"entry failure omitted same-tick stop");
            require(actor.calls==0&&mRobot->trajectory_attempts==0,"failed entry still ran/published in same FSMUpdate");++cases;
        }
    }

    void legacy_ankle_checks()
    {
        // The actual original AB2PitchRoll implementation is linked and called
        // for healthy legs. Only the negative-return case is injected.
        for(int failing_leg=-1;failing_leg<2;++failing_leg)
        {
            fixture();enter();
            const double raw[4]={.13,-.08,-.11,.06};
            const double velocity[4]={.2,-.3,.4,-.5};
            const int indices[4]={4,5,16,17};
            for(int i=0;i<4;++i)
            {mRobot->joints[indices[i]].pos_fdb=raw[i];mRobot->joints[indices[i]].vel_fdb=velocity[i];}
            double expected[2][4]{},J[2][2];
            for(int leg=0;leg<2;++leg)
            {
                const double sign=leg==0?1.:-1.;
                require(__real_AB2PitchRoll(sign*raw[2*leg],sign*raw[2*leg+1],
                    sign*velocity[2*leg],sign*velocity[2*leg+1],
                    &expected[leg][0],&expected[leg][1],&expected[leg][2],&expected[leg][3],J)>=0,
                    "legacy healthy ankle fixture rejected");
                if(leg==failing_leg)std::fill(std::begin(expected[leg]),std::end(expected[leg]),0.);
            }
            legacy_ankle_test::calls.clear();legacy_ankle_test::fail_call=failing_leg;
            fsm_mimic_jaka_mini_superzmq_run();
            require(legacy_ankle_test::calls.size()==2,"actual FSM did not call legacy AB C ABI twice");
            for(int leg=0;leg<2;++leg)
            {
                const auto &call=legacy_ankle_test::calls[leg];const double sign=leg==0?1.:-1.;
                require(call.a==sign*raw[2*leg]&&call.b==sign*raw[2*leg+1]&&
                    call.da==sign*velocity[2*leg]&&call.db==sign*velocity[2*leg+1],
                    "legacy left/right ankle input sign changed");
                const int d=indices[2*leg];
                require(std::abs(zmq_q[d]-expected[leg][0])<1.e-12&&
                    std::abs(zmq_q[d+1]-expected[leg][1])<1.e-12&&
                    std::abs(zmq_qd[d]-expected[leg][2])<1.e-12&&
                    std::abs(zmq_qd[d+1]-expected[leg][3])<1.e-12,
                    "legacy PR/velocity conversion or negative-return zero behavior changed");
            }
            require(mErrCode==ERR_NONE&&actor.calls==1&&mRobot->trajectory_accepted==1&&
                mRobot->pd_off==0&&mRobot->enable_off==0,
                "legacy negative ankle return must zero only that leg and continue policy");
            ++cases;
        }
    }

    void reference_reset_checks()
    {
        fixture();enter();actor.action.fill(.1f);fsm_mimic_jaka_mini_superzmq_run();
        require(mErrCode==ERR_NONE&&mRobot->trajectory_accepted==1,"healthy empty-buffer balance step failed");
        for(int i=485;i<620;++i)require(input[i]==0,"entry action history not zero");
        require(!zmq_first_step&&!zmq_last_ready,"empty-buffer first tick latch wrong");++cases;
        actor.action.fill(.2f);fsm_mimic_jaka_mini_superzmq_run();
        for(int i=593;i<620;++i)require(std::abs(input[i]-.1f)<1.e-7,"continuation lost raw previous action");
        ++cases;
        const auto old_history=zmq_obs_history;
        RealtimeMotionBuffer::next_ready=true;
        for(auto &frame:RealtimeMotionBuffer::next)
        {
            frame.root_quat_w=Eigen::Quaternionf(Eigen::AngleAxisf(.7f,Eigen::Vector3f::UnitZ()));
            frame.root_pos_w.z()=.72f;
        }
        actor.action.fill(.3f);fsm_mimic_jaka_mini_superzmq_run();
        require(zmq_last_ready,"first received legacy frame did not become ready immediately");
        require(std::abs(zmq_align_quat.z()+std::sin(.35f))<1.e-6,"first ready did not realign yaw");
        require(zmq_obs_history.size()==5,"first-ready history size changed");
        for(int h=0;h<4;++h)require(zmq_obs_history[h]==old_history[h+1],"first ready discarded running sensor/action history");
        for(int i=566;i<593;++i)require(std::abs(input[i]-.1f)<1.e-7,"first ready erased preceding action history");
        for(int i=593;i<620;++i)require(std::abs(input[i]-.2f)<1.e-7,"first ready reset raw previous action");
        require(std::abs(input[15]-.72f)<1.e-7,"first frame was delayed by an extra warmup");++cases;
        const auto aligned=zmq_align_quat;
        for(auto &frame:RealtimeMotionBuffer::next)
            frame.root_quat_w=Eigen::Quaternionf(Eigen::AngleAxisf(1.1f,Eigen::Vector3f::UnitZ()));
        fsm_mimic_jaka_mini_superzmq_run();
        require(zmq_align_quat.coeffs()==aligned.coeffs(),"ready continuation repeatedly realigned yaw");
        for(int i=593;i<620;++i)require(std::abs(input[i]-.3f)<1.e-7,"ready continuation lost raw action history");
        require(RealtimeMotionBuffer::ready_calls==4&&RealtimeMotionBuffer::query_calls==4,
            "actual FSM did not query original ready/get_obs API every tick");++cases;

        // If the first frame is already present on the first policy tick,
        // initialize once using that frame rather than requiring an extra tick.
        fixture();enter();RealtimeMotionBuffer::next_ready=true;
        for(auto &frame:RealtimeMotionBuffer::next)
            frame.root_quat_w=Eigen::Quaternionf(Eigen::AngleAxisf(-.4f,Eigen::Vector3f::UnitZ()));
        fsm_mimic_jaka_mini_superzmq_run();
        require(zmq_last_ready&&std::abs(zmq_align_quat.z()-std::sin(.2f))<1.e-6,
            "first policy tick with ready frame failed initial yaw alignment");
        for(int i=485;i<620;++i)require(input[i]==0,"first ready policy tick did not zero initial action history");
        ++cases;
    }

}

int main()
{
    try
    {
        transition_checks();fault_checks();legacy_ankle_checks();reference_reset_checks();
        zmq_motion_buffer.reset();zmq_state_pub.reset();mRobot.reset();
        std::cout<<"Actual IDLE/SuperMimic FSM transition, fault and legacy ankle/buffer cases passed: "<<cases<<"; fake I/O only\n";
        return 0;
    }
    catch(const std::exception &error)
    {std::cerr<<"Actual FSM test failed: "<<error.what()<<'\n';return 1;}
}
