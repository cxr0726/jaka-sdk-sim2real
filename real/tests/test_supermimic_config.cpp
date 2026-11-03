// Run the actual configuration initializer only. No RobotIntf/ROS/socket/FSM entry.
#include "RLController.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    void check(bool value,const char *why)
    {if(!value)throw std::runtime_error(why);}
}

int main(int argc,char **argv)
{
    try
    {
        check(argc==2,"usage: RealSuperMimicConfigTest config.yaml");
        using namespace RLController;
        const auto cfg=YAML::LoadFile(argv[1]);
        KpiConfigInit(argv[1]);
        check(!mRobot,"config initialization constructed hardware bridge");
        check(mFSMState==FSM_INIT && mEnabledJointCount==0,"config initialization entered/enabled control");
        check(mJakaMiniSuperZmqEnabled && bool(mJakaMiniSuperDeployment),"SuperMimic model was not loaded");
        check(mKp==cfg["kp"].as<std::vector<double>>(),"metadata overwrote hardware Kp");
        check(mKd==cfg["kd"].as<std::vector<double>>(),"metadata overwrote hardware Kd");
        check(mKp_PR==cfg["kp_pitch_roll"].as<std::vector<double>>(),"metadata overwrote PR Kp");
        check(mKd_PR==cfg["kd_pitch_roll"].as<std::vector<double>>(),"metadata overwrote PR Kd");
        check(mDefaultPos==cfg["default_angles"].as<std::vector<double>>(),"metadata overwrote SDK defaults");
        check(mInitTarget==cfg["init_target"].as<std::vector<double>>(),"metadata overwrote real initial targets");
        check(mAngleMapFlag==0 && std::abs(mTimeStep-.02)<1.e-12,"real control configuration changed");
        check(mMimicJakaMiniSuperZmqPolicy.input_shape[1]==620 &&
              mMimicJakaMiniSuperZmqPolicy.output_shape[1]==27,"actor shape mismatch");
        const int indices[4]={4,5,16,17};
        const auto lower=cfg["pitch_roll_down_limit"].as<std::vector<double>>();
        const auto upper=cfg["pitch_roll_up_limit"].as<std::vector<double>>();
        for(int i=0;i<4;++i)
            check(mDownLmt[indices[i]]==lower[i] && mUpLmt[indices[i]]==upper[i],"physical PR limits not loaded from YAML");
        KpiConfigDeinit();
        check(!mJakaMiniSuperDeployment && !mMimicJakaMiniSuperZmqPolicy.session,"SuperMimic config resources not released");
        std::cout<<"Actual KpiConfigInit retained all real YAML gains/defaults/limits; hardware never constructed\n";
        return 0;
    }
    catch(const std::exception &error)
    {std::cerr<<error.what()<<'\n';return 1;}
}
