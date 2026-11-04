#include "CheckedMujocoPhysics.hpp"
#include "MujocoRobotBackend.hpp"
#include "ParKine.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool value,const char *message) {if(!value)throw std::runtime_error(message);}
MujocoRobotBackend::Config configuration(const char *xml) {
    MujocoRobotBackend::Config c;c.xml_path=xml;c.angle_map=0;c.init_base_pos={0.,0.,.65};
    c.joint_names={"left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint",
      "left_ankle_pitch_joint","left_ankle_roll_joint","left_shoulder_pitch_joint","left_shoulder_roll_joint",
      "left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint","left_wrist_yaw_joint",
      "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint","right_knee_joint",
      "right_ankle_pitch_joint","right_ankle_roll_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint",
      "right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_yaw_joint",
      "waist_yaw_joint","neck_yaw_joint","neck_pitch_joint"};
    c.init_target.assign(27,0.);c.kp.assign(27,0.);c.kd.assign(27,0.);
    c.kp[4]=c.kp[16]=.01;c.kp[5]=c.kp[17]=.02;
    c.kp_pitch_roll={.01,.02,.01,.02};c.kd_pitch_roll={0.,0.,0.,0.};return c;
}
int qaddress(const mjModel *m,const char *name) {
    const int id=mj_name2id(m,mjOBJ_JOINT,name);require(id>=0,"drive joint missing");return m->jnt_qposadr[id];
}
void enable(MujocoRobotBackend &b) {b.start();b.do_enable(true);b.enable_pd(true);}
void check_shutdown(MujocoRobotBackend &b) {
    for(int i=0;i<b.model()->nu;++i)require(b.data()->ctrl[i]==0.,"fault retained actuator command");
    legged_waist_controller::RobotIntf::JointStatus status{};
    require(b.get_joint_status("left_ankle_pitch_joint",status)>=0&&!status.is_enable,"fault retained drive enable");
}
void sdk_uses_original_abi(const char *xml,int side) {
    auto c=configuration(xml);MujocoRobotBackend b(c);enable(b);
    // Isolate one PD evaluation: no contact/equality impulses or gravity are
    // needed to prove which kinematics the precompiled SDK actually calls.
    b.model()->opt.disableflags|=mjDSBL_CONSTRAINT|mjDSBL_GRAVITY;
    std::fill_n(b.data()->qvel,b.model()->nv,0.);
    constexpr double a=-.7943700221369686,beta=1.6945090391329518;
    double pitch,roll,dpitch,droll,J[2][2];
    require(AB2PitchRoll(a,beta,0.,0.,&pitch,&roll,&dpitch,&droll,J)>=0,"legacy ABI fixture failed");
    const char *first=side==0?"Left_drive1_joint":"Right_drive1_joint";
    const char *second=side==0?"Left_drive2_joint":"Right_drive2_joint";
    const double sign=side==0?1.:-1.;
    b.data()->qpos[qaddress(b.model(),first)]=sign*a;
    b.data()->qpos[qaddress(b.model(),second)]=sign*beta;
    mj_forward(b.model(),b.data());
    jaka::CheckedMujocoPhysics guard(b);require(guard.step(),guard.error().c_str());
    const double tp=-.01*pitch,tr=-.02*roll;
    const double wanted[2]={sign*(J[0][0]*tp+J[1][0]*tr),
                            sign*(J[0][1]*tp+J[1][1]*tr)};
    const char *motors[2]={side==0?"Left_drive1_motor":"Right_drive1_motor",
                           side==0?"Left_drive2_motor":"Right_drive2_motor"};
    for(int i=0;i<2;++i) {
        const int id=mj_name2id(b.model(),mjOBJ_ACTUATOR,motors[i]);require(id>=0,"drive actuator missing");
        if(std::abs(b.data()->ctrl[id]-wanted[i])>1.e-10) {
            std::cerr<<motors[i]<<" ctrl="<<b.data()->ctrl[id]<<" expected="<<wanted[i]<<'\n';
            throw std::runtime_error("SDK PD did not use original ABI root/Jacobian (ELF preemption/sign regression)");
        }
    }
    require(guard.successful_steps()==1,"healthy substep count");guard.shutdown();check_shutdown(b);
}
void nonfinite_input_stops_sdk(const char *xml,int which) {
    MujocoRobotBackend b(configuration(xml));enable(b);jaka::CheckedMujocoPhysics guard(b);
    const double time=b.simulation_time();
    std::fill_n(b.data()->ctrl,b.model()->nu,1.);
    const double nan=std::numeric_limits<double>::quiet_NaN();
    if(which==0)b.data()->qvel[0]=nan;
    else if(which==1)b.data()->qpos[0]=nan;
    else b.data()->ctrl[0]=nan;
    require(!guard.step(),"nonfinite input entered SDK physics");
    require(guard.successful_steps()==0&&b.simulation_time()==time,"fault advanced physics before rejection");
    require(!guard.error().empty(),"fault diagnostic missing");check_shutdown(b);
    require(!guard.step()&&b.simulation_time()==time,"fault was not latched");
}
void finite_state_has_no_new_assembly_gate(const char *xml) {
    auto config=configuration(xml);
    std::fill(config.kp.begin(),config.kp.end(),0.);
    std::fill(config.kp_pitch_roll.begin(),config.kp_pitch_roll.end(),0.);
    MujocoRobotBackend b(config);enable(b);
    b.model()->opt.disableflags|=mjDSBL_CONSTRAINT|mjDSBL_GRAVITY;
    b.data()->qpos[qaddress(b.model(),"Left_drive1_joint")]=1.5;
    b.data()->qpos[qaddress(b.model(),"Left_drive2_joint")]=1.5;
    mj_forward(b.model(),b.data());
    jaka::CheckedMujocoPhysics guard(b);
    const double time=b.simulation_time();
    require(guard.step(),"finite legacy AB state was blocked by an assembly preflight");
    require(guard.successful_steps()==1&&b.simulation_time()>time,"finite SDK step did not advance");
    guard.shutdown();check_shutdown(b);
}
}
int main(int argc,char **argv) {
    try {require(argc==2,"expected XML path");sdk_uses_original_abi(argv[1],0);sdk_uses_original_abi(argv[1],1);
        for(int which=0;which<3;++which)nonfinite_input_stops_sdk(argv[1],which);
        finite_state_has_no_new_assembly_gate(argv[1]);
        std::cout<<"original SDK PD ABI/Jacobian parity, finite legacy state and generic nonfinite isolation passed\n";return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
