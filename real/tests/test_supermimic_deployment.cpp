#include "SuperMimicDeployment.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void check(bool passed, const std::string &message)
{
    if (!passed) throw std::runtime_error(message);
}

void rejects(const std::function<void()> &operation, const std::string &message)
{
    try { operation(); }
    catch (const std::exception &) { return; }
    throw std::runtime_error("expected rejection: " + message);
}

// ModelProto field 14 is metadata_props. Remove only these top-level fields
// in memory: graph/tensors stay byte-identical and no Python ONNX is needed.
uint64_t varint(const std::vector<char> &bytes, size_t &cursor)
{
    uint64_t result=0;
    for (int shift=0; shift<64; shift+=7)
    {
        check(cursor<bytes.size(), "truncated protobuf varint");
        const auto byte=static_cast<unsigned char>(bytes[cursor++]);
        result|=uint64_t(byte&0x7f)<<shift;
        if (!(byte&0x80)) return result;
    }
    throw std::runtime_error("oversized protobuf varint");
}

std::vector<char> without_metadata(const std::vector<char> &bytes)
{
    std::vector<char> result;
    size_t cursor=0, removed=0;
    while (cursor<bytes.size())
    {
        const size_t start=cursor;
        const uint64_t tag=varint(bytes,cursor);
        const unsigned wire=unsigned(tag&7);
        const uint64_t field=tag>>3;
        if (wire==0) (void)varint(bytes,cursor);
        else
        {
            uint64_t length=0;
            if (wire==1) length=8;
            else if (wire==5) length=4;
            else if (wire==2) length=varint(bytes,cursor);
            else throw std::runtime_error("unsupported top-level protobuf wire type");
            check(length<=bytes.size()-cursor, "truncated protobuf field");
            cursor+=size_t(length);
        }
        if (field==14) ++removed;
        else result.insert(result.end(),bytes.begin()+start,bytes.begin()+cursor);
    }
    check(removed>0, "actual model unexpectedly has no metadata fixture");
    return result;
}

using Contract=RLController::SuperMimicDeployment;
const std::array<int,27> expected_deploy={
    0,1,2,3,4,5,12,13,14,15,16,17,24,6,7,8,9,10,11,18,19,20,21,22,23,25,26};
const std::array<int,27> expected_buffer={
    0,3,8,13,17,21,1,4,9,14,18,22,2,5,10,15,19,23,25,7,12,16,20,24,26,6,11};

void check_mapping(const Contract &contract, const std::vector<double> &defaults)
{
    check(contract.policy_to_deploy==expected_deploy,"policy to SDK name mapping");
    check(contract.policy_from_buffer==expected_buffer,"policy to buffer name mapping");
    const auto buffer=contract.buffer_default_joint_pos();
    for (int p=0;p<27;++p)
    {
        const float expected=static_cast<float>(defaults[expected_deploy[p]]);
        check(contract.default_joint_pos[p]==expected,"defaults must come from supplied SDK-order YAML values");
        check(buffer[expected_buffer[p]]==expected,"standing fallback must use the same YAML defaults in buffer order");
    }
}
} // namespace

int main(int argc,char **argv)
{
    try
    {
        check(argc==3,"usage: RealSuperMimicDeploymentTest <super.onnx> <KpiConfig.yaml>");
        const YAML::Node yaml=YAML::LoadFile(argv[2]);
        const auto names=yaml["joint_names"].as<std::vector<std::string>>();
        const auto defaults=yaml["default_angles"].as<std::vector<double>>();
        const double fallback=yaml["action_scale"].as<double>();
        const YAML::Node cfg=YAML::Clone(yaml["jakaminimimic_superzmq"]);
        check(cfg.IsMap(),"real SuperMimic section must be present");
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"real-supermimic-yaml-test");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);options.SetInterOpNumThreads(1);
        Ort::Session session(env,argv[1],options);
        const Contract actual(session,cfg,names,defaults,fallback);
        check_mapping(actual,defaults);
        check(actual.angular_velocity_scale==.5f&&actual.joint_velocity_scale==.2f,
              "actual real YAML must retain SuperMimic sensor scales");
        for (float scale:actual.action_scale) check(scale==.5f,"actual real YAML action scale");

        // Deliberately contradict the actor's metadata with unique YAML defaults.
        // Every joint should retain its own name-associated sentinel throughout.
        std::vector<double> sentinel(27);
        for (int i=0;i<27;++i) sentinel[i]=.03125*(i+1);
        const Contract sentinels(session,cfg,names,sentinel,fallback);
        check_mapping(sentinels,sentinel);

        auto scalar=YAML::Clone(cfg);
        scalar["action_scale"]=.375;
        scalar["ang_vel_scale"]=.625;
        scalar["dof_vel_scale"]=.25;
        const Contract overrides(session,scalar,names,sentinel,fallback);
        for (float scale:overrides.action_scale) check(scale==.375f,"scalar YAML action override");
        check(overrides.angular_velocity_scale==.625f&&overrides.joint_velocity_scale==.25f,
              "sensor scales must come from YAML rather than metadata or generic constants");

        auto per_joint=YAML::Clone(cfg);
        std::vector<double> scales(27);
        for (int p=0;p<27;++p) scales[p]=.015625*(p+1);
        per_joint["action_scale"]=scales;
        const Contract vector_scale(session,per_joint,names,sentinel,fallback);
        for (int p=0;p<27;++p)
            check(vector_scale.action_scale[p]==static_cast<float>(scales[p]),"per-policy-joint YAML action scale");
        auto inherited=YAML::Clone(cfg);inherited.remove("action_scale");
        const Contract global_scale(session,inherited,names,sentinel,.3125);
        for (float scale:global_scale.action_scale) check(scale==.3125f,"fallback generic YAML action scale");

        // The real adapter must work even when the ONNX carries no metadata.
        // This catches accidental regression to sim's model-owned parameters.
        std::ifstream input(argv[1],std::ios::binary);
        check(bool(input),"cannot read actual ONNX fixture");
        const std::vector<char> bytes((std::istreambuf_iterator<char>(input)),{});
        const auto stripped=without_metadata(bytes);
        Ort::Session metadata_free(env,stripped.data(),stripped.size(),options);
        const Contract no_metadata(metadata_free,per_joint,names,sentinel,fallback);
        check_mapping(no_metadata,sentinel);
        check(no_metadata.action_scale==vector_scale.action_scale,"metadata-free model must keep YAML scales");

        auto missing=YAML::Clone(cfg);missing.remove("joint_names");
        rejects([&]{Contract bad(session,missing,names,defaults,fallback);},"missing policy joint_names");
        auto duplicate=YAML::Clone(cfg);duplicate["joint_names"][1]=duplicate["joint_names"][0].as<std::string>();
        rejects([&]{Contract bad(session,duplicate,names,defaults,fallback);},"duplicate policy joint name");
        auto unknown=YAML::Clone(cfg);unknown["joint_names"][0]="missing_joint";
        rejects([&]{Contract bad(session,unknown,names,defaults,fallback);},"unknown policy joint name");
        auto short_names=YAML::Clone(cfg);
        auto policy_names=cfg["joint_names"].as<std::vector<std::string>>();policy_names.pop_back();short_names["joint_names"]=policy_names;
        rejects([&]{Contract bad(session,short_names,names,defaults,fallback);},"26 policy joints");
        auto missing_sdk=names;missing_sdk.pop_back();
        rejects([&]{Contract bad(session,cfg,missing_sdk,defaults,fallback);},"26 SDK joints");
        auto short_defaults=defaults;short_defaults.pop_back();
        rejects([&]{Contract bad(session,cfg,names,short_defaults,fallback);},"26 SDK default angles");
        auto nan_defaults=defaults;nan_defaults[9]=std::numeric_limits<double>::quiet_NaN();
        rejects([&]{Contract bad(session,cfg,names,nan_defaults,fallback);},"nonfinite YAML default angle");
        for (double invalid:{0.,-.5,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
        {
            auto invalid_scale=YAML::Clone(cfg);invalid_scale["action_scale"]=invalid;
            rejects([&]{Contract bad(session,invalid_scale,names,defaults,fallback);},"invalid scalar action scale");
        }
        auto short_scales=YAML::Clone(cfg);short_scales["action_scale"]=std::vector<double>(26,.5);
        rejects([&]{Contract bad(session,short_scales,names,defaults,fallback);},"26 action scales");
        auto bad_sensor=YAML::Clone(cfg);bad_sensor["dof_vel_scale"]=-.2;
        rejects([&]{Contract bad(session,bad_sensor,names,defaults,fallback);},"negative joint velocity scale");
        auto bad_angular=YAML::Clone(cfg);bad_angular["ang_vel_scale"]=std::numeric_limits<double>::quiet_NaN();
        rejects([&]{Contract bad(session,bad_angular,names,defaults,fallback);},"nonfinite angular velocity scale");

        std::cout<<"Real SuperMimic YAML defaults/scales, name mappings, metadata independence and invalid-config rejection passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
