#include "SuperMimicDeployment.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <nlohmann/json.hpp>

namespace RLController
{
    namespace
    {
        using json = nlohmann::json;
        constexpr int kJoints = SuperMimicDeployment::kNumJoints;
        constexpr std::array<int, kJoints> kLegacyIsaacToDeploy = {
            0, 12, 24, 1, 13, 6, 25, 18, 2, 14, 7, 26, 19, 3,
            15, 8, 20, 4, 16, 9, 21, 5, 17, 10, 22, 11, 23};
        constexpr std::array<const char *, kJoints> kLegacyDeployNames = {
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint",
            "waist_yaw_joint", "neck_yaw_joint", "neck_pitch_joint"};

        [[noreturn]] void fail(const std::string &message)
        {
            throw std::runtime_error("SuperMimic deployment contract: " + message);
        }

        std::string normalized_name(std::string value)
        {
            const auto not_space = [](unsigned char c) { return !std::isspace(c); };
            value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
            value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
            std::transform(value.begin(), value.end(), value.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return value;
        }

        std::vector<std::string> csv(const std::string &value)
        {
            std::vector<std::string> result;
            std::istringstream input(value);
            std::string item;
            while (std::getline(input, item, ',')) result.push_back(item);
            if (!value.empty() && value.back() == ',') result.emplace_back();
            return result;
        }

        std::string metadata(Ort::ModelMetadata &model, Ort::AllocatorWithDefaultOptions &allocator,
                             const char *key)
        {
            const auto value = model.LookupCustomMetadataMapAllocated(key, allocator);
            if (!value || !*value.get()) fail(std::string("missing ONNX metadata '") + key + "'");
            return value.get();
        }

        std::array<float, kJoints> numeric_metadata(
            Ort::ModelMetadata &model, Ort::AllocatorWithDefaultOptions &allocator,
            const char *key, bool positive, bool nonnegative)
        {
            const auto values = csv(metadata(model, allocator, key));
            if (values.size() != kJoints) fail(std::string(key) + " must contain 27 values");
            std::array<float, kJoints> result{};
            for (int i = 0; i < kJoints; ++i)
            {
                const auto value = normalized_name(values[i]);
                size_t consumed = 0;
                float number = 0;
                try { number = std::stof(value, &consumed); }
                catch (const std::exception &) { fail(std::string(key) + " contains an invalid number"); }
                if (consumed != value.size() || !std::isfinite(number) ||
                    (positive && number <= 0) || (nonnegative && number < 0))
                    fail(std::string(key) + " contains a nonfinite or unsupported value at index " + std::to_string(i));
                result[i] = number;
            }
            return result;
        }

        void require(const json &object, const char *key, const json &expected,
                     const std::string &scope)
        {
            const auto found = object.find(key);
            if (found == object.end() || *found != expected)
                fail(scope + "." + key + " must be " + expected.dump());
        }

        void validate_terms(const json &schema, const char *key,
                            const std::vector<std::string> &names,
                            const std::vector<int> &sizes, const std::vector<float> &scales,
                            int packed_offset)
        {
            const auto found = schema.find(key);
            if (found == schema.end() || !found->is_array() || found->size() != names.size())
                fail(std::string("schema.") + key + " has an incompatible term list");
            int start = 0;
            for (size_t i = 0; i < names.size(); ++i)
            {
                const auto &term = (*found)[i];
                const auto scope = std::string("schema.") + key + "[" + std::to_string(i) + "]";
                const int stop = start + sizes[i] * 5;
                require(term, "name", names[i], scope);
                require(term, "start", start, scope);
                require(term, "stop", stop, scope);
                require(term, "packed_start", packed_offset + start, scope);
                require(term, "packed_stop", packed_offset + stop, scope);
                require(term, "per_frame_dim", sizes[i], scope);
                require(term, "frames", 5, scope);
                // Parse as double to avoid comparing JSON 0.2 against float's
                // 0.20000000298 representation.
                if (!term.contains("caller_scale") || !term["caller_scale"].is_number() ||
                    std::abs(term["caller_scale"].get<double>() - scales[i]) > 1.e-7)
                    fail(scope + ".caller_scale is incompatible");
                start = stop;
            }
        }

        void validate_schema(const json &schema)
        {
            require(schema, "version", 1, "schema");
            require(schema, "format", "supermimic.khan.actor", "schema");
            require(schema, "robot", "khan_mini", "schema");
            require(schema, "input_layout", "packed", "schema");
            require(schema, "observation_layout", "isaac_khan", "schema");
            require(schema, "dimensions", {{"actions", 27}, {"goal", 155}, {"obs", 620}, {"policy_obs", 465}}, "schema");
            require(schema, "future_frames", 5, "schema");
            require(schema, "history", 5, "schema");
            require(schema, "future_offsets", {0, 1, 2, 3, 4}, "schema");
            require(schema, "history_offsets", {-4, -3, -2, -1, 0}, "schema");
            require(schema, "history_layout", "oldest_to_newest_per_term", "schema");
            require(schema, "history_reset", "repeat first sensor sample five times; previous action zero", "schema");
            require(schema, "packed_order", {"goal", "policy_obs"}, "schema");
            require(schema, "future_orientation_history", false, "schema");
            require(schema, "quaternion_order", "wxyz", "schema");
            require(schema, "rotation_6d_order", {"R00", "R01", "R10", "R11", "R20", "R21"}, "schema");
            require(schema, "root_displacement", "R_ref(t)^T @ (p_ref(t+k)-p_ref(t)); full reference rotation", "schema");
            require(schema, "reference_joint_pos", "absolute radians, not relative to default_joint_pos", "schema");
            require(schema, "joint_pos", "measured radians minus default_joint_pos, in joint_names order", "schema");
            require(schema, "projected_gravity", "R_base^T @ [0,0,-1]", "schema");
            require(schema, "angular_velocity_frame", "base_link body frame, radians/second", "schema");
            require(schema, "orientation", "R_base(t)^T @ R_ref(t+k)", "schema");
            require(schema, "sensor_scale_location", "caller, once; network scale buffers remain identity", "schema");
            const auto running_normalization = schema.find("running_normalization");
            if (running_normalization == schema.end() || !running_normalization->is_boolean())
                fail("schema.running_normalization must be a boolean");
            if (running_normalization->get<bool>())
            {
                // Official actors keep their frozen running statistics and the
                // inverse of the legacy caller sensor scales inside ONNX. The
                // caller still supplies omega*0.5 and qd*0.2 exactly once; do
                // not normalize, unscale or update statistics in this process.
                const auto normalization = schema.find("normalization");
                if (normalization == schema.end() || !normalization->is_object())
                    fail("schema.normalization must declare embedded frozen normalization");
                require(*normalization, "embedded_in_onnx", true, "schema.normalization");
                require(*normalization, "updates_at_inference", false, "schema.normalization");
                require(*normalization, "formula", "(raw_feature - running_mean) / (running_std + 0.01)", "schema.normalization");
                require(*normalization, "khan_sensor_input", "caller omega*0.5 and qd*0.2 are inverted inside the graph before normalization", "schema.normalization");
                require(*normalization, "sensor_history_statistics", "one shared statistic per raw sensor feature across history lags", "schema.normalization");
                require(*normalization, "future_orientation_statistics", "independent flattened 30-dimensional window", "schema.normalization");
                require(*normalization, "goal_statistics", "independent per flattened goal dimension and future offset", "schema.normalization");
            }
            require(schema, "observation_noise", false, "schema");
            require(schema, "actor_stateful", false, "schema");
            require(schema, "reference_motion_embedded", false, "schema");
            require(schema, "control_dt", 0.02, "schema");
            require(schema, "control_frequency_hz", 50, "schema");
            if (!schema.contains("action") || !schema["action"].is_object()) fail("schema.action is missing");
            const auto &action = schema["action"];
            require(action, "meaning", "raw normalized joint-position action", "schema.action");
            require(action, "clip", nullptr, "schema.action");
            require(action, "tanh", false, "schema.action");
            require(action, "smoothing", false, "schema.action");
            require(action, "execution_noise", false, "schema.action");
            require(action, "pd_target", "default_joint_pos + action_scale * actions", "schema.action");
            require(action, "last_action", "previous raw actions, not PD target or scaled action", "schema.action");
            validate_terms(schema, "goal_terms", {"reference_root_displacement_b", "reference_world_z", "reference_joint_pos"},
                           {3, 1, 27}, {1.f, 1.f, 1.f}, 0);
            validate_terms(schema, "policy_terms", {"motion_anchor_ori_b", "projected_gravity", "base_ang_vel", "joint_pos", "joint_vel", "actions"},
                           {6, 3, 3, 27, 27, 27}, {1.f, 1.f, .5f, 1.f, .2f, 1.f}, 155);
        }
    } // namespace

    SuperMimicDeployment::SuperMimicDeployment(Ort::Session &session,
                                               const std::vector<std::string> &deploy_names)
    {
        if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
            fail("expected one packed observation input and one action output");
        Ort::AllocatorWithDefaultOptions allocator;
        const auto input_name = session.GetInputNameAllocated(0, allocator);
        const auto output_name = session.GetOutputNameAllocated(0, allocator);
        if (std::string(input_name.get()) != "obs" || std::string(output_name.get()) != "actions")
            fail("tensor names must be obs and actions");
        const auto input_type = session.GetInputTypeInfo(0);
        const auto output_type = session.GetOutputTypeInfo(0);
        const auto input = input_type.GetTensorTypeAndShapeInfo();
        const auto output = output_type.GetTensorTypeAndShapeInfo();
        const auto input_shape = input.GetShape();
        const auto output_shape = output.GetShape();
        if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            input_shape.size() != 2 || output_shape.size() != 2 ||
            (input_shape[0] != -1 && input_shape[0] != 1) ||
            (output_shape[0] != -1 && output_shape[0] != 1) ||
            input_shape[1] != 620 || output_shape[1] != kJoints)
            fail("expected float32 obs[batch,620] -> actions[batch,27], batch dynamic or 1");

        auto model = session.GetModelMetadata();
        const std::array<std::pair<const char *, const char *>, 8> required_metadata = {{
            {"robot", "khan_mini"}, {"input_layout", "packed"},
            {"observation_layout", "isaac_khan"}, {"anchor_body_name", "base_link"},
            {"root_body_name", "base_link"}, {"observation_history", "5"},
            {"goal_frames", "5"}, {"control_dt", "0.02"}}};
        for (const auto &[key, expected] : required_metadata)
            if (metadata(model, allocator, key) != expected)
                fail(std::string("metadata '") + key + "' must be '" + expected + "'");
        try { validate_schema(json::parse(metadata(model, allocator, "onnx_deployment_schema"))); }
        catch (const json::exception &error) { fail(std::string("invalid onnx_deployment_schema: ") + error.what()); }

        if (deploy_names.size() != kJoints) fail("deploy joint_names must contain 27 names");
        std::unordered_map<std::string, int> deploy_index;
        std::unordered_map<std::string, int> buffer_index;
        for (int i = 0; i < kJoints; ++i)
        {
            const auto name = normalized_name(deploy_names[i]);
            if (name != kLegacyDeployNames[i])
                fail("legacy deployment joint order mismatch at index " + std::to_string(i) +
                     ": expected " + kLegacyDeployNames[i] + ", got " + name);
            deploy_index.emplace(name, i);
        }
        for (int i = 0; i < kJoints; ++i)
            buffer_index.emplace(normalized_name(deploy_names[kLegacyIsaacToDeploy[i]]), i);
        const auto policy_names = csv(metadata(model, allocator, "joint_names"));
        if (policy_names.size() != kJoints) fail("metadata joint_names must contain 27 names");
        std::unordered_set<std::string> seen;
        for (int i = 0; i < kJoints; ++i)
        {
            const auto name = normalized_name(policy_names[i]);
            if (!seen.emplace(name).second) fail("duplicate policy joint name: " + name);
            if (deploy_index.count(name) == 0 || buffer_index.count(name) == 0)
                fail("unknown policy joint name: " + name);
            policy_to_deploy[i] = deploy_index.at(name);
            policy_from_buffer[i] = buffer_index.at(name);
        }
        default_joint_pos = numeric_metadata(model, allocator, "default_joint_pos", false, false);
        action_scale = numeric_metadata(model, allocator, "action_scale", true, false);
        joint_stiffness = numeric_metadata(model, allocator, "joint_stiffness", false, true);
        joint_damping = numeric_metadata(model, allocator, "joint_damping", false, true);
    }

    Eigen::VectorXf SuperMimicDeployment::buffer_default_joint_pos() const
    {
        Eigen::VectorXf result(kNumJoints);
        for (int policy = 0; policy < kNumJoints; ++policy)
            result[policy_from_buffer[policy]] = default_joint_pos[policy];
        return result;
    }
} // namespace RLController
