#include "VlaPosePacket.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

namespace
{
    using Json = nlohmann::json;
    using Bytes = std::vector<std::uint8_t>;
    using RLController::VlaPosePacket;

    void check(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    template <typename Uint>
    void append_le(Bytes &bytes, Uint value)
    {
        for (std::size_t i = 0; i < sizeof(value); ++i)
            bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
    }

    void append_float(Bytes &bytes, float value)
    {
        std::uint32_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        append_le(bytes, bits);
    }

    struct Fixture
    {
        Json header = {{"v", 1}, {"endian", "le"}, {"count", 1}, {"anchor_body", "base_link"},
                       {"joint_order", "jaka"}, {"stream_id", "demo-process"}, {"fields", Json::array()}};
        Bytes payload;

        Fixture(std::uint64_t second_index = 101, bool include_velocity = true, float last_velocity = 0.f)
        {
            // Field order may vary. The payload follows the declared order.
            auto field = [&](const char *name, const char *dtype, const Json &shape) {
                header["fields"].push_back({{"name", name}, {"dtype", dtype}, {"shape", shape}});
            };
            field("frame_index", "i64", {2});
            append_le(payload, std::uint64_t{100});
            append_le(payload, second_index);
            field("joint_pos", "f32", {2, 27});
            for (int i = 0; i < 54; ++i)
                append_float(payload, static_cast<float>(i) / 50.f);
            field("body_quat_w", "f32", {2, 4});
            for (int i = 0; i < 2; ++i)
                for (float q : {0.601f, 0.f, 0.f, 0.8f})
                    append_float(payload, q);
            field("body_pos_w", "f32", {2, 3});
            for (float x : {1.f, 2.f, 0.65f, 1.02f, 2.f, 0.66f})
                append_float(payload, x);
            if (include_velocity)
            {
                field("joint_vel", "f32", {2, 27});
                for (int i = 0; i < 54; ++i)
                    append_float(payload, i == 53 ? last_velocity : 0.f);
            }
        }

        Bytes bytes() const
        {
            Bytes result{'p', 'o', 's', 'e'};
            const auto text = header.dump();
            check(text.size() < 1280, "fixture header too long");
            result.insert(result.end(), text.begin(), text.end());
            result.resize(4 + 1280, 0);
            result.insert(result.end(), payload.begin(), payload.end());
            return result;
        }
    };

    void check_output(const VlaPosePacket &value)
    {
        check(value.stream_id == "demo-process" && value.frames.size() == 2, "wrong packet identity");
        for (int row = 0; row < 2; ++row)
        {
            const auto &frame = value.frames[row];
            check(frame.index == 100 + row, "wrong index");
            for (int joint = 0; joint < 27; ++joint)
                check(frame.joints[joint] == static_cast<float>(27 * row + joint) / 50.f,
                      "wrong row or JAKA wire joint order");
            check(frame.position[0] == (row == 0 ? 1.f : 1.02f) && frame.position[1] == 2.f &&
                      frame.position[2] == (row == 0 ? 0.65f : 0.66f), "wrong position");
            const auto &q = frame.quaternion;
            check(std::abs(q[0] * q[0] + q[3] * q[3] - 1.f) < 1e-6f &&
                      q[1] == 0.f && q[2] == 0.f && q[0] > 0.60f && q[3] > 0.79f,
                  "wrong wxyz quaternion or normalization");
        }
    }
}

int main()
{
    VlaPosePacket accepted;
    for (bool include_velocity : {true, false})
    {
        const auto bytes = Fixture(101, include_velocity).bytes();
        check(RLController::decode_vla_pose(bytes.data(), bytes.size(), accepted), "valid packet rejected");
        check_output(accepted);
    }

    auto rejected = [&](const Bytes &bytes) {
        // Make an accidental partial commit observable even if the first row itself was valid.
        accepted.stream_id = "previous-stream";
        accepted.frames[0].index = 900;
        check(!RLController::decode_vla_pose(bytes.data(), bytes.size(), accepted), "invalid packet accepted");
        check(accepted.stream_id == "previous-stream" && accepted.frames.size() == 2 &&
                  accepted.frames[0].index == 900, "partial write on failure");
        accepted.stream_id = "demo-process";
        accepted.frames[0].index = 100;
        check_output(accepted);
    };
    Fixture legacy;
    for (const char *key : {"anchor_body", "joint_order", "stream_id"})
    {
        auto missing = legacy;
        missing.header.erase(key);
        rejected(missing.bytes());
    }
    legacy.header["anchor_body"] = "waist_yaw_Link";
    rejected(legacy.bytes());
    Fixture wrong_axis;
    wrong_axis.header["joint_order"] = "isaac";
    rejected(wrong_axis.bytes());
    Fixture wrong_shape;
    wrong_shape.header["fields"][1]["shape"] = {2, 26};
    rejected(wrong_shape.bytes());
    Fixture wrong_dtype;
    wrong_dtype.header["fields"][0]["dtype"] = "i32";
    rejected(wrong_dtype.bytes());
    rejected(Fixture(100).bytes());
    rejected(Fixture(105).bytes());
    rejected(Fixture(101, true, std::numeric_limits<float>::quiet_NaN()).bytes());
    auto truncated = Fixture().bytes();
    truncated.pop_back();
    rejected(truncated);
    auto extra = Fixture().bytes();
    extra.push_back(0);
    rejected(extra);
    std::cout << "JAKA pose wire layout, frame identity, optional velocity and atomic rejection passed\n";
}
