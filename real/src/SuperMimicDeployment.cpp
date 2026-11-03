#include "SuperMimicDeployment.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace RLController
{
    namespace
    {
        constexpr int kJoints=SuperMimicDeployment::kNumJoints;
        constexpr std::array<int,kJoints> kIsaacToDeploy={
            0,12,24,1,13,6,25,18,2,14,7,26,19,3,15,8,20,4,16,9,21,5,17,10,22,11,23};
        constexpr std::array<const char *,kJoints> kDeployNames={
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
            "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
            "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
            "left_elbow_joint","left_wrist_roll_joint","left_wrist_yaw_joint",
            "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
            "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
            "right_elbow_joint","right_wrist_roll_joint","right_wrist_yaw_joint",
            "waist_yaw_joint","neck_yaw_joint","neck_pitch_joint"};

        [[noreturn]] void fail(const std::string &why)
        {throw std::runtime_error("Real SuperMimic YAML contract: "+why);}

        std::string normalized(std::string value)
        {
            const auto nonspace=[](unsigned char c){return !std::isspace(c);};
            value.erase(value.begin(),std::find_if(value.begin(),value.end(),nonspace));
            value.erase(std::find_if(value.rbegin(),value.rend(),nonspace).base(),value.end());
            std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){return std::tolower(c);});
            return value;
        }

        float positive_scale(double value,const char *field)
        {
            const float result=static_cast<float>(value);
            if(!std::isfinite(value)||!std::isfinite(result)||result<=0)
                fail(std::string(field)+" must contain finite positive values");
            return result;
        }
    }

    SuperMimicDeployment::SuperMimicDeployment(
        Ort::Session &session,const YAML::Node &config,
        const std::vector<std::string> &deploy_names,
        const std::vector<double> &deploy_default_pos,double fallback_action_scale)
    {
        // Inspect tensor names/types/shapes only. In particular, do not import
        // metadata Kp/Kd, defaults, scales or joint names into this real process.
        if(session.GetInputCount()!=1||session.GetOutputCount()!=1)
            fail("expected one packed input and one action output");
        Ort::AllocatorWithDefaultOptions allocator;
        const auto input_name=session.GetInputNameAllocated(0,allocator);
        const auto output_name=session.GetOutputNameAllocated(0,allocator);
        const auto input_type=session.GetInputTypeInfo(0);
        const auto output_type=session.GetOutputTypeInfo(0);
        const auto input=input_type.GetTensorTypeAndShapeInfo();
        const auto output=output_type.GetTensorTypeAndShapeInfo();
        const auto in=input.GetShape(),out=output.GetShape();
        if(std::string(input_name.get())!="obs"||std::string(output_name.get())!="actions"||
           input.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||
           output.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||
           in.size()!=2||out.size()!=2||(in[0]!=-1&&in[0]!=1)||(out[0]!=-1&&out[0]!=1)||
           in[1]!=620||out[1]!=27)
            fail("expected float32 obs[batch,620] -> actions[batch,27]");
        if(config["observation_layout"].as<std::string>("")!="isaac_khan")
            fail("observation_layout must explicitly be isaac_khan");
        if(deploy_names.size()!=kJoints||deploy_default_pos.size()!=kJoints)
            fail("global joint_names/default_angles must contain 27 entries");
        const auto names=config["joint_names"];
        if(!names||!names.IsSequence()||names.size()!=kJoints)
            fail("jakaminimimic_superzmq.joint_names must specify all 27 policy joints");
        std::unordered_map<std::string,int> deploy_index,buffer_index;
        for(int i=0;i<kJoints;++i)
        {
            const auto name=normalized(deploy_names[i]);
            if(name!=kDeployNames[i])fail("global joint_names differs from SDK deployment order at "+std::to_string(i));
            deploy_index.emplace(name,i);
            buffer_index.emplace(normalized(deploy_names[kIsaacToDeploy[i]]),i);
        }
        std::unordered_set<std::string> seen;
        for(int i=0;i<kJoints;++i)
        {
            const auto name=normalized(names[i].as<std::string>());
            if(!seen.emplace(name).second)fail("duplicate policy joint "+name);
            if(!deploy_index.count(name)||!buffer_index.count(name))fail("unknown policy joint "+name);
            policy_to_deploy[i]=deploy_index.at(name);
            policy_from_buffer[i]=buffer_index.at(name);
            const double q=deploy_default_pos[policy_to_deploy[i]];
            default_joint_pos[i]=static_cast<float>(q);
            if(!std::isfinite(q)||!std::isfinite(default_joint_pos[i]))fail("nonfinite default_angles");
        }
        const auto scale=config["action_scale"];
        if(scale&&scale.IsSequence())
        {
            if(scale.size()!=kJoints)fail("action_scale sequence must have 27 policy-order entries");
            for(int i=0;i<kJoints;++i)action_scale[i]=positive_scale(scale[i].as<double>(),"action_scale");
        }
        else
            action_scale.fill(positive_scale(scale?scale.as<double>():fallback_action_scale,"action_scale"));
        angular_velocity_scale=positive_scale(config["ang_vel_scale"].as<double>(),"ang_vel_scale");
        joint_velocity_scale=positive_scale(config["dof_vel_scale"].as<double>(),"dof_vel_scale");
    }

    Eigen::VectorXf SuperMimicDeployment::buffer_default_joint_pos() const
    {
        Eigen::VectorXf result(kNumJoints);
        for(int p=0;p<kNumJoints;++p)result[policy_from_buffer[p]]=default_joint_pos[p];
        return result;
    }
}
