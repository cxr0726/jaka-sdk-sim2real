#include "CheckedMujocoPhysics.hpp"
#include "MujocoRobotBackend.hpp"
#include "ParallelAnkleSolver.hpp"
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
void sdk_uses_checked_root(const char *xml,int side) {
    auto c=configuration(xml);MujocoRobotBackend b(c);enable(b);
    // Isolate one PD evaluation: no contact/equality impulses or gravity are
    // needed to prove which kinematics the precompiled SDK actually calls.
    b.model()->opt.disableflags|=mjDSBL_CONSTRAINT|mjDSBL_GRAVITY;
    std::fill_n(b.data()->qvel,b.model()->nv,0.);
    constexpr double a=-.7943700221369686,beta=1.6945090391329518;
    const auto expected=jaka::solve_parallel_ankle(a,beta,0.,0.);require(expected.ok(),"recorded root fixture invalid");
    const char *first=side==0?"Left_drive1_joint":"Right_drive1_joint";
    const char *second=side==0?"Left_drive2_joint":"Right_drive2_joint";
    const double sign=side==0?1.:-1.;
    b.data()->qpos[qaddress(b.model(),first)]=sign*a;
    b.data()->qpos[qaddress(b.model(),second)]=sign*beta;
    mj_forward(b.model(),b.data());
    jaka::CheckedMujocoPhysics guard(b);require(guard.step(),guard.error().c_str());
    const double tp=-.01*expected.pitch,tr=-.02*expected.roll;
    const double wanted[2]={sign*(expected.J[0][0]*tp+expected.J[1][0]*tr),
                            sign*(expected.J[0][1]*tp+expected.J[1][1]*tr)};
    const char *motors[2]={side==0?"Left_drive1_motor":"Right_drive1_motor",
                           side==0?"Left_drive2_motor":"Right_drive2_motor"};
    for(int i=0;i<2;++i) {
        const int id=mj_name2id(b.model(),mjOBJ_ACTUATOR,motors[i]);require(id>=0,"drive actuator missing");
        if(std::abs(b.data()->ctrl[id]-wanted[i])>1.e-10) {
            std::cerr<<motors[i]<<" ctrl="<<b.data()->ctrl[id]<<" expected="<<wanted[i]<<'\n';
            throw std::runtime_error("SDK PD did not use checked root/Jacobian (ELF preemption/sign regression)");
        }
    }
    require(guard.successful_steps()==1,"healthy substep count");guard.shutdown();check_shutdown(b);
}
void preflight_failure_stops_sdk(const char *xml,bool finite_failure) {
    MujocoRobotBackend b(configuration(xml));enable(b);jaka::CheckedMujocoPhysics guard(b);
    const double time=b.simulation_time();
    if(finite_failure) {
        bool found=false;
        for(const auto &v:std::array<std::array<double,2>,4>{{{{1.5,1.5}},{{-2.,-2.}},{{3.,3.}},{{1.,1.}}}})
            if(!jaka::solve_parallel_ankle(v[0],v[1],0.,0.).ok()) {
                b.data()->qpos[qaddress(b.model(),"Left_drive1_joint")]=v[0];
                b.data()->qpos[qaddress(b.model(),"Left_drive2_joint")]=v[1];found=true;break;
            }
        require(found,"no finite invalid assembly fixture");
    } else b.data()->qvel[0]=std::numeric_limits<double>::quiet_NaN();
    std::fill_n(b.data()->ctrl,b.model()->nu,1.);
    require(!guard.step(),"invalid input entered SDK physics");
    require(guard.successful_steps()==0&&b.simulation_time()==time,"fault advanced physics before rejection");
    require(!guard.error().empty(),"fault diagnostic missing");check_shutdown(b);
    require(!guard.step()&&b.simulation_time()==time,"fault was not latched");
}
}
int main(int argc,char **argv) {
    try {require(argc==2,"expected XML path");sdk_uses_checked_root(argv[1],0);sdk_uses_checked_root(argv[1],1);
        preflight_failure_stops_sdk(argv[1],false);preflight_failure_stops_sdk(argv[1],true);
        std::cout<<"real SDK PD root/Jacobian parity and preflight fault isolation passed\n";return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
