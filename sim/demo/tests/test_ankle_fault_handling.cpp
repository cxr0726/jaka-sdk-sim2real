#include "RLController.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace RLController
{
std::unique_ptr<legged_waist_controller::RobotIntf> mRobot;
ERRCODE mErrCode = ERR_NONE;
int mNumAction = 27, mEnabledJointCount = 0, mFaultJointCount = 0;
bool mDataReadyFlag = true;
int mAngleMapFlag = 0;
std::vector<std::string> mJointNames(27);
NLF2 mNlf2[50]{};
}

using Robot = legged_waist_controller::RobotIntf;
struct SpyBackend final : Robot::Backend
{
    bool enabled = true, pd = true;
    int bad_joint = -1, commands = 0;
    int start() override { return 0; }
    int stop() override { return 0; }
    int do_enable(bool value) override { enabled = value; return 0; }
    int enable_pd(bool value) override { pd = value; return 0; }
    int set_joint_operation_mode(std::int8_t) override { return 0; }
    int set_joint_mit_gains(const std::string &, double, double) override { return 0; }
    int set_joint_pd_para(const std::string &, double, double) override { return 0; }
    int set_joint_angle_limit(const std::string &, double, double) override { return 0; }
    int get_joint_status(const std::string &name, Robot::JointStatus &out) const override
    {
        const int index = std::stoi(name);
        if (index == bad_joint) return -1;
        out = {}; out.is_enable = enabled; out.pos_fdb = .01*index;
        return 0;
    }
    bool has_imu() const override { return false; }
    bool get_imu(Robot::ImuData &) const override { return false; }
    bool get_joy_robot_state(Robot::JoyRobotState &) const override { return false; }
    int set_joint_trajectory(const std::vector<std::string> &,
                            const trajectory_msgs::msg::JointTrajectoryPoint &) override
    { ++commands; return 0; }
};

static void require(bool ok, const char *why)
{
    if (!ok) throw std::runtime_error(why);
}

int main()
{
    try
    {
        using namespace RLController;
        Robot::Options options; options.enable_joy_sub = false;
        mRobot = std::make_unique<Robot>(options);
        auto backend = std::make_shared<SpyBackend>(); mRobot->attach_backend(backend);
        for (int i = 0; i < 27; ++i) mJointNames[i] = std::to_string(i);
        std::array<double, 50> q{}, qd{};
        double a,b,lp,lr,ldp,ldr,rp,rr,rdp,rdr,J[2][2];
        PitchRoll2AB(.1,-.05,&a,&b);q[4]=a;q[5]=b;
        PitchRoll2AB(-.2,.1,&a,&b);q[16]=-a;q[17]=-b;
        qd[4]=.2;qd[5]=-.3;qd[16]=.4;qd[17]=-.5;q[8]=.123;
        require(AB2PitchRoll(q[4],q[5],qd[4],qd[5],&lp,&lr,&ldp,&ldr,J)>=0,"left fixture");
        require(AB2PitchRoll(-q[16],-q[17],-qd[16],-qd[17],&rp,&rr,&rdp,&rdr,J)>=0,"right fixture");
        reset_parallel_ankle_observers();
        require(decode_parallel_ankles(q.data(),qd.data(),"legacy healthy"),"valid decode");
        require(q[4]==lp&&q[5]==lr&&qd[4]==ldp&&qd[5]==ldr&&
                q[16]==rp&&q[17]==rr&&qd[16]==rdp&&qd[17]==rdr,"original ABI/sign/velocity parity");
        require(q[8]==.123&&backend->enabled&&backend->pd&&mErrCode==ERR_NONE,"healthy side effects");

        // A negative result has the original per-leg zero fallback. It must
        // neither latch the removed assembly fault nor discard the good leg.
        PitchRoll2AB(.1,-.05,&q[4],&q[5]);
        q[16]=std::numeric_limits<double>::quiet_NaN();q[17]=0.;
        require(decode_parallel_ankles(q.data(),qd.data(),"legacy bad right"),"legacy failure was escalated");
        require(std::abs(q[4]-.1)<2.e-5&&std::abs(q[5]+.05)<2.e-5,"good leg lost on right failure");
        require(q[16]==0.&&q[17]==0.&&qd[16]==0.&&qd[17]==0.,"negative return did not zero only failed leg");
        require(mErrCode==ERR_NONE&&backend->pd&&backend->enabled,"negative ABI return disabled controller");
        require(backend->commands==0,"decoder emitted a trajectory");

        std::vector<double> targets(27,0.);targets[4]=.1;targets[5]=-.05;
        targets[16]=-.2;targets[17]=.1;targets[8]=.123;
        double la,lb,ra,rb;PitchRoll2AB(.1,-.05,&la,&lb);PitchRoll2AB(-.2,.1,&ra,&rb);
        require(encode_parallel_ankle_targets(targets,"legacy target"),"legacy inverse rejected");
        require(targets[4]==la&&targets[5]==lb&&targets[16]==-ra&&targets[17]==-rb&&targets[8]==.123,
                "original inverse/right target signs changed");

        backend->enabled=true;backend->bad_joint=9;
        update_joint_status(q.data(),q.data(),qd.data());
        require(mFaultJointCount==1 && !std::isfinite(q[9]), "failed feedback read reused a previous joint");
        mErrCode=ERR_NONE; backend->enabled=backend->pd=true;
        for (int i=0;i<27;++i) {mNlf2[i].x0=42.+i;mNlf2[i].v0=-42.-i;}
        bind_nlf2_state();
        require(mErrCode==ERR_JOINT && !backend->pd && !backend->enabled, "planner feedback failure not disabled/latched");
        for (int i=0;i<27;++i)
            require(mNlf2[i].x0==42.+i && mNlf2[i].v0==-42.-i, "partially bound invalid planner state");
        mRobot.reset();
        std::cout << "legacy ankle feedback/target parity, zero fallback and generic I/O faults passed\n";
        return 0;
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
