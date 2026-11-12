#include "SuperMimicDeployment.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace
{
    void check(bool passed, const std::string &message)
    {
        if (!passed) throw std::runtime_error(message);
    }

    void rejects(const std::function<void()> &operation, const std::string &message_fragment)
    {
        try { operation(); }
        catch (const std::runtime_error &error)
        {
            check(std::string(error.what()).find(message_fragment) != std::string::npos,
                  "unexpected rejection: " + std::string(error.what()));
            return;
        }
        throw std::runtime_error("expected rejection containing: " + message_fragment);
    }

    std::vector<char> replace_once(std::vector<char> bytes, const std::string &from, const std::string &to)
    {
        check(from.size() == to.size(), "protobuf fixture substitutions must preserve length");
        const auto begin = std::search(bytes.begin(), bytes.end(), from.begin(), from.end());
        check(begin != bytes.end(), "metadata fixture text not found: " + from);
        std::copy(to.begin(), to.end(), begin);
        return bytes;
    }
}

int main(int argc, char **argv)
{
    try
    {
        check(argc == 3 || argc == 4,
              "usage: SuperMimicDeploymentTest <super.onnx> <legacy.onnx> [official.onnx]");
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "supermimic-contract-test");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        const std::vector<std::string> names = {
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
            "left_ankle_pitch_joint", "left_ankle_roll_joint", "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_yaw_joint", "right_hip_pitch_joint",
            "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint",
            "right_ankle_roll_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
            "right_wrist_yaw_joint", "waist_yaw_joint", "neck_yaw_joint", "neck_pitch_joint"};
        Ort::Session session(env, argv[1], options);
        const RLController::SuperMimicDeployment contract(session, names);
        const std::array<int, 27> expected_deploy = {
            0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 24, 6, 7, 8, 9, 10, 11,
            18, 19, 20, 21, 22, 23, 25, 26};
        const std::array<int, 27> expected_buffer = {
            0, 3, 8, 13, 17, 21, 1, 4, 9, 14, 18, 22, 2, 5, 10, 15, 19, 23, 25,
            7, 12, 16, 20, 24, 26, 6, 11};
        check(contract.policy_to_deploy == expected_deploy, "actual actor deployment joint mapping");
        check(contract.policy_from_buffer == expected_buffer, "actual actor motion-buffer joint mapping");
        std::array<float, 27> scattered{};
        const auto defaults = contract.buffer_default_joint_pos();
        for (int i = 0; i < 27; ++i)
        {
            scattered[contract.policy_to_deploy[i]] = 1000.f + i;
            check(defaults[expected_buffer[i]] == contract.default_joint_pos[i], "default motion reorder");
            check(contract.action_scale[i] == .5f, "actual checkpoint action scale");
        }
        check(scattered[12] == 1006.f && scattered[24] == 1012.f &&
              scattered[6] == 1013.f && scattered[25] == 1025.f, "action scattering sentinels");
        check(std::abs(defaults[10] + 1.57f) < 1.e-6f &&
              std::abs(defaults[19] - 1.57f) < 1.e-6f &&
              std::abs(defaults[25] - .3f) < 1.e-6f && defaults[0] == 0,
              "legacy Isaac defaults retain shoulder/elbow/wrist semantics");
        check(contract.joint_stiffness[0] == 187.f && contract.joint_stiffness[4] == 100.f &&
              contract.joint_stiffness[5] == 50.f && contract.joint_damping[5] == .5f,
              "actual checkpoint leg PD metadata");
        check(contract.angular_velocity_scale == .5f && contract.joint_velocity_scale == .2f,
              "SuperMimic sensor scales");

        auto mixed_case = names;
        mixed_case[0] = "Left_hip_pitch_joint";
        check(RLController::SuperMimicDeployment(session, mixed_case).policy_to_deploy == expected_deploy,
              "case-insensitive joint mapping");
        auto wrong_order = names;
        std::swap(wrong_order[0], wrong_order[1]);
        rejects([&] { RLController::SuperMimicDeployment bad(session, wrong_order); }, "legacy deployment joint order mismatch");
        auto missing = names;
        missing.pop_back();
        rejects([&] { RLController::SuperMimicDeployment bad(session, missing); }, "27 names");

        // The legacy actor also has 620 inputs and 27 outputs: dimensions alone
        // cannot distinguish its waist anchor, Isaac order and normalizer.
        Ort::Session legacy(env, argv[2], options);
        rejects([&] { RLController::SuperMimicDeployment bad(legacy, names); }, "ONNX metadata");

        std::ifstream stream(argv[1], std::ios::binary);
        check(static_cast<bool>(stream), "cannot read actual model for in-memory metadata tests");
        const std::vector<char> bytes((std::istreambuf_iterator<char>(stream)), {});
        const auto normalized = replace_once(bytes, "\"running_normalization\": false", "\"running_normalization\": true ");
        Ort::Session invalid_schema(env, normalized.data(), normalized.size(), options);
        rejects([&] { RLController::SuperMimicDeployment bad(invalid_schema, names); }, "schema.normalization");
        const auto clipped = replace_once(bytes, "\"clip\": null", "\"clip\": 5.0 ");
        Ort::Session invalid_clip(env, clipped.data(), clipped.size(), options);
        rejects([&] { RLController::SuperMimicDeployment bad(invalid_clip, names); }, "schema.action.clip");

        // Preserve the protobuf field length while poisoning one scalar in the
        // actual metadata. No inference or file writes are needed for this case.
        const auto nonfinite = replace_once(bytes, "187,187,187,187,100,50", "nan,187,187,187,100,50");
        Ort::Session invalid_gain(env, nonfinite.data(), nonfinite.size(), options);
        rejects([&] { RLController::SuperMimicDeployment bad(invalid_gain, names); }, "joint_stiffness contains a nonfinite");

        if (argc == 4)
        {
            // Official EMA has frozen running normalization inside the graph,
            // while preserving the same raw, caller-scaled input contract.
            Ort::Session official(env, argv[3], options);
            const RLController::SuperMimicDeployment official_contract(official, names);
            check(official_contract.policy_to_deploy == expected_deploy &&
                  official_contract.policy_from_buffer == expected_buffer,
                  "official actor preserves all joint mappings");
            check(official_contract.default_joint_pos == contract.default_joint_pos &&
                  official_contract.action_scale == contract.action_scale,
                  "official actor preserves defaults and action scales");
            check(official_contract.joint_stiffness[4] == 80.f &&
                  official_contract.joint_stiffness[5] == 20.f &&
                  official_contract.joint_damping[4] == 2.f &&
                  official_contract.joint_damping[5] == 1.f,
                  "official actor ankle PD metadata");
            std::ifstream official_stream(argv[3], std::ios::binary);
            check(static_cast<bool>(official_stream), "cannot read official model");
            const std::vector<char> official_bytes((std::istreambuf_iterator<char>(official_stream)), {});
            for (const auto &entry : std::vector<std::array<std::string, 3>>{
                     {"\"embedded_in_onnx\": true", "\"embedded_in_onnx\": null", "embedded_in_onnx"},
                     {"\"updates_at_inference\": false", "\"updates_at_inference\": true ", "updates_at_inference"},
                     {"running_std + 0.01", "running_std + 0.02", "formula"},
                     {"caller omega*0.5 and qd*0.2", "caller omega*1.0 and qd*1.0", "khan_sensor_input"}})
            {
                const auto invalid = replace_once(official_bytes, entry[0], entry[1]);
                Ort::Session broken(env, invalid.data(), invalid.size(), options);
                rejects([&] { RLController::SuperMimicDeployment bad(broken, names); },
                        "schema.normalization." + entry[2]);
            }
            const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            std::array<float, 620> observation{};
            const std::array<int64_t, 2> shape{1, 620};
            auto input = Ort::Value::CreateTensor<float>(memory, observation.data(), observation.size(),
                                                        shape.data(), shape.size());
            const char *input_name = "obs", *output_name = "actions";
            auto output = official.Run(Ort::RunOptions{nullptr}, &input_name, &input, 1, &output_name, 1);
            check(output[0].GetTensorTypeAndShapeInfo().GetElementCount() == 27,
                  "official graph output size");
            const float *actions = output[0].GetTensorData<float>();
            check(std::all_of(actions, actions + 27, [](float value) { return std::isfinite(value); }),
                  "official embedded normalizer produces finite actions");
        }
        std::cout << "SuperMimic deployment metadata, mappings and rejection checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
