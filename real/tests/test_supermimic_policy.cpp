// Offline exercise of the ACTUAL new FSM's math. No RobotIntf construction,
// ROS initialization, ZMQ buffer construction, servo command or FSM entry.
#include "../src/FSMMimicJakaMiniSuperZmq.cpp"

#include <limits>
#include <stdexcept>

namespace RLController
{
    // Only satisfy the unused exit dispatcher when linking this standalone TU.
    void FSMIdleEnter(FSMSTATE) {}
    void FSMFaultEnter(FSMSTATE) {}
}

namespace
{
    using namespace RLController;
    constexpr std::array<int,27> kExpectedDeploy={
        0,1,2,3,4,5,12,13,14,15,16,17,24,6,7,8,9,10,11,18,19,20,21,22,23,25,26};
    constexpr std::array<int,27> kExpectedBuffer={
        0,3,8,13,17,21,1,4,9,14,18,22,2,5,10,15,19,23,25,7,12,16,20,24,26,6,11};

    void check(bool ok,const char *message)
    {if(!ok)throw std::runtime_error(message);}
    void near(double actual,double expected,const char *message,double tolerance=1.e-5)
    {check(std::isfinite(actual)&&std::abs(actual-expected)<=tolerance,message);}

    std::array<InterpolatedFrame,5> frames_fixture()
    {
        std::array<InterpolatedFrame,5> frames;
        for(int f=0;f<5;++f)
        {
            frames[f].root_pos_w={1.f+.04f*f,2.f+.01f*f*f,.6f+.025f*f};
            frames[f].root_quat_w=Eigen::Quaternionf(
                Eigen::AngleAxisf(.1f*f,Eigen::Vector3f::UnitZ())*
                Eigen::AngleAxisf(.4f,Eigen::Vector3f::UnitY())*
                Eigen::AngleAxisf(-.3f,Eigen::Vector3f::UnitX()));
            frames[f].dof_pos.resize(27);
            for(int j=0;j<27;++j)frames[f].dof_pos[j]=1000.f*f+10.f*j+.25f;
        }
        return frames;
    }

    std::vector<float> sample_fixture(int step)
    {
        std::vector<float> expected(87);
        zmq_quat[0]=1;zmq_quat[1]=zmq_quat[2]=zmq_quat[3]=0;
        expected[0]=0;expected[1]=0;expected[2]=-1;
        for(int axis=0;axis<3;++axis)
        {
            zmq_omega[axis]=step+.2*axis;
            expected[3+axis]=static_cast<float>(zmq_omega[axis]*mJakaMiniSuperDeployment->angular_velocity_scale);
        }
        for(int p=0;p<27;++p)
        {
            const int d=kExpectedDeploy[p];
            const float offset=.001f*(100*step+p);
            zmq_q[d]=static_cast<double>(mJakaMiniSuperDeployment->default_joint_pos[p])+offset;
            zmq_qd[d]=-.2+.01*d+.05*step;
            zmq_last_action[p]=-1.f+.1f*step+.01f*p;
            expected[6+p]=offset;
            expected[33+p]=static_cast<float>(zmq_qd[d]*mJakaMiniSuperDeployment->joint_velocity_scale);
            expected[60+p]=zmq_last_action[p];
        }
        zmq_construct_single_obs();
        for(int i=0;i<87;++i)near(zmq_single_obs[i],expected[i],"current observation/default/scaling/joint map");
        return expected;
    }

    void pure_math_checks()
    {
        check(zmq_resolve_policy_layout(),"cannot resolve actual FSM policy layout");
        check(mJakaMiniSuperDeployment->policy_to_deploy==kExpectedDeploy,"YAML policy -> SDK permutation");
        check(mJakaMiniSuperDeployment->policy_from_buffer==kExpectedBuffer,"YAML policy -> buffer permutation");
        const auto frames=frames_fixture();
        const auto command=zmq_construct_command(frames);
        const Eigen::Matrix3f ref_rotation=frames[0].root_quat_w.toRotationMatrix();
        for(int f=0;f<5;++f)
        {
            const Eigen::Vector3f displacement=ref_rotation.transpose()*(frames[f].root_pos_w-frames[0].root_pos_w);
            for(int axis=0;axis<3;++axis)near(command[3*f+axis],displacement[axis],"full reference rotation displacement");
            near(command[15+f],frames[f].root_pos_w[2],"absolute reference world Z");
            for(int p=0;p<27;++p)
                near(command[20+27*f+p],1000.f*f+10.f*kExpectedBuffer[p]+.25f,"frame-major absolute reference q");
        }
        zmq_quat[0]=1;zmq_quat[1]=zmq_quat[2]=zmq_quat[3]=0;
        zmq_align_quat=Eigen::Quaternionf(Eigen::AngleAxisf(.6f,Eigen::Vector3f::UnitZ()));
        const auto ori=zmq_construct_anchor_orientation(frames);
        for(int f=0;f<5;++f)
        {
            const Eigen::Matrix3f expected=zmq_align_quat.toRotationMatrix()*frames[f].root_quat_w.toRotationMatrix();
            for(int row=0;row<3;++row)for(int col=0;col<2;++col)
                near(ori[6*f+2*row+col],expected(row,col),"future orientation 6d row/column order");
        }
        std::fill(zmq_last_action.begin(),zmq_last_action.end(),123.f);
        zmq_initialize_last_action();
        check(std::all_of(zmq_last_action.begin(),zmq_last_action.end(),[](float x){return x==0;}),"raw previous action reset zero");
        std::deque<std::vector<float>> history;
        for(int step=0;step<7;++step)
        {
            const auto expected=sample_fixture(step);
            if(step==0){zmq_init_history();history.assign(5,expected);}
            else {zmq_push_history();history.pop_front();history.push_back(expected);}
            zmq_pack_history(frames);
            for(int i=0;i<155;++i)near(zmq_stacked_obs[i],command[i],"goal packing offset");
            for(int i=0;i<30;++i)near(zmq_stacked_obs[155+i],ori[i],"future orientation packing offset");
            constexpr std::array<int,5> size={3,3,27,27,27};
            int read=0,write=185;
            for(const int width:size)
            {
                for(const auto &old:history)for(int j=0;j<width;++j)
                    near(zmq_stacked_obs[write++],old[read+j],"oldest-to-newest per-term history packing");
                read+=width;
            }
            check(write==620,"packed actor observation length");
        }

        mAngleMapFlag=0;
        for(int p=0;p<27;++p)mMimicJakaMiniSuperZmqPolicy.output_buffer[p]=.01f*p-.13f;
        mMimicJakaMiniSuperZmqPolicy.output_buffer[0]=12.5f;
        check(zmq_fill_trajectory(),"finite policy target rejected");
        for(int p=0;p<27;++p)
        {
            const int d=kExpectedDeploy[p];
            const float raw=mMimicJakaMiniSuperZmqPolicy.output_buffer[p];
            const double target=static_cast<double>(raw)*mJakaMiniSuperDeployment->action_scale[p]+mJakaMiniSuperDeployment->default_joint_pos[p];
            near(mPt.positions[d],std::clamp(target,mDownLmt[d],mUpLmt[d]),"real target default/scale/scatter/physical limit");
            near(zmq_last_action[p],raw,"previous action must retain raw value despite target limiting");
        }
        near(zmq_last_action[0],12.5,"no inherited raw action +/-5 clipping");
        const auto old_target=mPt.positions;
        const auto old_action=zmq_last_action;
        mMimicJakaMiniSuperZmqPolicy.output_buffer[17]=std::numeric_limits<float>::quiet_NaN();
        check(!zmq_fill_trajectory(),"nonfinite raw action accepted");
        check(old_target==mPt.positions&&old_action==zmq_last_action,"nonfinite action partially committed targets/history");
    }

    MotionWindow read_window(const nlohmann::json &row)
    {
        MotionWindow window;
        window.query_steady_ns=row.at("query_steady_ns").get<int64_t>();
        window.generation=row.at("generation").get<uint64_t>();
        const auto status=row.at("status").get<std::string>();
        if(status=="tracking")window.status=MotionWindow::Status::Tracking;
        else if(status=="underrun")window.status=MotionWindow::Status::Underrun;
        else if(status=="stale")window.status=MotionWindow::Status::Stale;
        for(int f=0;f<5;++f)
        {
            const auto &source=row.at("frames").at(f);
            const auto pos=source.at("root_pos").get<std::vector<float>>();
            const auto quat=source.at("root_quat").get<std::vector<float>>();
            const auto q=source.at("joint_pos_buffer").get<std::vector<float>>();
            check(pos.size()==3&&quat.size()==4&&q.size()==27,"invalid replay reference shape");
            window.frames[f].root_pos_w={pos[0],pos[1],pos[2]};
            window.frames[f].root_quat_w={quat[0],quat[1],quat[2],quat[3]};
            window.frames[f].dof_pos=Eigen::Map<const Eigen::VectorXf>(q.data(),27);
        }
        return window;
    }

    void replay_tape(const char *input,const char *output,size_t maximum)
    {
        std::ifstream in(input);check(static_cast<bool>(in),"cannot open source parity tape");
        zmq_policy_diagnostic.open(output);check(static_cast<bool>(zmq_policy_diagnostic),"cannot open real parity tape");
        zmq_timing_step_count=0;zmq_obs_history.clear();
        std::string line;
        while(zmq_timing_step_count<maximum&&std::getline(in,line))
        {
            const auto row=nlohmann::json::parse(line);
            const auto q=row.at("q_deploy").get<std::vector<double>>();
            const auto qd=row.at("qd_deploy").get<std::vector<double>>();
            const auto omega=row.at("omega").get<std::vector<double>>();
            const auto quat=row.at("robot_quat").get<std::vector<double>>();
            check(q.size()==27&&qd.size()==27&&omega.size()==3&&quat.size()==4,"invalid replay state shape");
            std::copy(q.begin(),q.end(),zmq_q);std::copy(qd.begin(),qd.end(),zmq_qd);
            std::copy(omega.begin(),omega.end(),zmq_omega);std::copy(quat.begin(),quat.end(),zmq_quat);
            const auto window=read_window(row);
            const bool reset=row.at("reset_history").get<bool>();
            check(zmq_timing_step_count>0||reset,"replay must begin at a history reset");
            if(reset)
            {
                zmq_initialize_alignment(window.frames[0]);
                zmq_initialize_last_action();zmq_construct_single_obs();zmq_init_history();
            }
            else {zmq_construct_single_obs();zmq_push_history();}
            zmq_pack_history(window.frames);
            mMimicJakaMiniSuperZmqPolicy.session->Run(Ort::RunOptions(nullptr),
                &mMimicJakaMiniSuperZmqPolicy.input_name,mMimicJakaMiniSuperZmqPolicy.input_tensor,1,
                &mMimicJakaMiniSuperZmqPolicy.output_name,mMimicJakaMiniSuperZmqPolicy.output_tensor,1);
            check(zmq_fill_trajectory(),"offline inference produced invalid targets");
            zmq_write_policy_diagnostic(window,reset);
            ++zmq_timing_step_count;
        }
        check(zmq_timing_step_count>0,"empty parity replay");
        zmq_policy_diagnostic.close();
        std::cout<<"Offline actual-FSM replay ticks: "<<zmq_timing_step_count<<" -> "<<output<<'\n';
    }
}

int main(int argc,char **argv)
{
    try
    {
        check(argc==3||argc==5||argc==6,"usage: RealSuperMimicPolicyTest config.yaml model.onnx [source.jsonl real.jsonl [max_steps]]");
        using namespace RLController;
        const auto yaml=YAML::LoadFile(argv[1]);
        mNumAction=27;mTimeStep=.02;mAngleMapFlag=0;
        mJointNames=yaml["joint_names"].as<std::vector<std::string>>();
        mDefaultPos=yaml["default_angles"].as<std::vector<double>>();
        mInitTarget=yaml["init_target"].as<std::vector<double>>();
        mKp=yaml["kp"].as<std::vector<double>>();mKd=yaml["kd"].as<std::vector<double>>();
        mKp_PR=yaml["kp_pitch_roll"].as<std::vector<double>>();mKd_PR=yaml["kd_pitch_roll"].as<std::vector<double>>();
        mDownLmt=yaml["down_limit"].as<std::vector<double>>();mUpLmt=yaml["up_limit"].as<std::vector<double>>();
        const auto pr_lower=yaml["pitch_roll_down_limit"].as<std::vector<double>>(
            std::vector<double>{-.52,-.35,-.52,-.35});
        const auto pr_upper=yaml["pitch_roll_up_limit"].as<std::vector<double>>(
            std::vector<double>{.61,.35,.61,.35});
        check(pr_lower.size()==4&&pr_upper.size()==4,"physical PR YAML limit shape");
        const int pr_indices[4]={4,5,16,17};
        for(int i=0;i<4;++i)
        {
            mDownLmt[pr_indices[i]]=pr_lower[i];
            mUpLmt[pr_indices[i]]=pr_upper[i];
        }
        const auto saved_kp=mKp,saved_kd=mKd,saved_init=mInitTarget;
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"real-supermimic-offline-test");
        Ort::SessionOptions options;options.SetIntraOpNumThreads(1);options.SetInterOpNumThreads(1);
        Ort::Session session(env,argv[2],options);
        mJakaMiniSuperDeployment=std::make_unique<SuperMimicDeployment>(session,yaml["jakaminimimic_superzmq"],
            mJointNames,mDefaultPos,yaml["action_scale"].as<double>());
        check(mKp==saved_kp&&mKd==saved_kd&&mInitTarget==saved_init,"model overwrote hardware YAML joint parameters");
        check(std::all_of(mKp.begin(),mKp.end(),[](double x){return x==100.;}),"hardware-converted Kp not retained");
        for(int i:{8,9,10,11,20,21,22,23,25,26})near(mKd[i],200.,"hardware-converted Kd not retained");
        std::array<float,620> input{};std::array<float,27> action{};
        std::array<int64_t,2> in_shape={1,620},out_shape={1,27};
        auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
        auto input_tensor=Ort::Value::CreateTensor<float>(memory,input.data(),input.size(),in_shape.data(),2);
        auto output_tensor=Ort::Value::CreateTensor<float>(memory,action.data(),action.size(),out_shape.data(),2);
        auto &ctx=mMimicJakaMiniSuperZmqPolicy;
        ctx.session=&session;ctx.input_name=const_cast<char *>("obs");ctx.output_name=const_cast<char *>("actions");
        ctx.input_shape[0]=1;ctx.input_shape[1]=620;ctx.output_shape[0]=1;ctx.output_shape[1]=27;
        ctx.input_buffer=input.data();ctx.output_buffer=action.data();ctx.input_tensor=&input_tensor;ctx.output_tensor=&output_tensor;
        mPt.positions.resize(27);
        pure_math_checks();
        check(!mRobot,"offline test unexpectedly constructed RobotIntf");
        if(argc>=5)replay_tape(argv[3],argv[4],argc==6?std::stoull(argv[5]):1600);
        mJakaMiniSuperDeployment.reset();
        ctx.session=nullptr;ctx.input_tensor=nullptr;ctx.output_tensor=nullptr;
        std::cout<<"Actual SuperMimic FSM observation/history/action checks passed; hardware bridge never constructed\n";
        return 0;
    }
    catch(const std::exception &error)
    {
        std::cerr<<error.what()<<'\n';
        return 1;
    }
}
