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
        double a, b;
        require(PitchRoll2ABChecked(.1, -.05, &a, &b) >= 0, "valid left inverse");
        q[4] = a; q[5] = b;
        require(PitchRoll2ABChecked(-.2, .1, &a, &b) >= 0, "valid right inverse");
        q[16] = -a; q[17] = -b; q[8] = .123;
        reset_parallel_ankle_observers();
        require(decode_parallel_ankles(q.data(), qd.data(), "unit healthy"), "valid decode");
        require(std::abs(q[4]-.1)<1.e-8 && std::abs(q[5]+.05)<1.e-8 &&
                std::abs(q[16]+.2)<1.e-8 && std::abs(q[17]-.1)<1.e-8, "joint order and right signs");
        require(q[8]==.123 && backend->enabled && backend->pd && mErrCode==ERR_NONE, "healthy conversion side effects");

        // A real old wrong-root input must stay in the physical assembly branch.
        q[16] = .7943700221369686; q[17] = -1.6945090391329518;
        require(PitchRoll2ABChecked(.1, -.05, &q[4], &q[5]) >= 0, "prepare raw feedback");
        require(decode_parallel_ankles(q.data(), qd.data(), "unit old wrong-root"), "recover old wrong root");
        require(std::abs(q[16]+.546446)<2.e-5 && std::abs(q[17]+.382821)<2.e-5, "correct assembly root");

        require(PitchRoll2ABChecked(.1, -.05, &q[4], &q[5]) >= 0, "prepare left");
        qd[16] = std::numeric_limits<double>::quiet_NaN();
        const auto before_q=q, before_qd=qd;
        require(!decode_parallel_ankles(q.data(), qd.data(), "unit bad right"), "invalid feedback accepted");
        require(std::memcmp(q.data(),before_q.data(),sizeof(q))==0 &&
                std::memcmp(qd.data(),before_qd.data(),sizeof(qd))==0, "half-converted feedback escaped");
        require(mErrCode==ERR_JOINT && !backend->pd && !backend->enabled, "feedback fault not disabled/latched");
        require(backend->commands==0, "fault emitted a trajectory");

        mErrCode=ERR_NONE;backend->enabled=backend->pd=true;
        std::vector<double> targets(27,0.);targets[16]=std::numeric_limits<double>::quiet_NaN();
        const auto before_targets=targets;
        require(!encode_parallel_ankle_targets(targets,"unit invalid inverse"), "invalid target accepted");
        require(std::memcmp(targets.data(),before_targets.data(),targets.size()*sizeof(double))==0,
                "half-converted inverse target escaped");
        require(mErrCode==ERR_JOINT && !backend->pd && !backend->enabled, "inverse fault not disabled/latched");

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
        std::cout << "checked ankle observation and target fault propagation passed\n";
        return 0;
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
