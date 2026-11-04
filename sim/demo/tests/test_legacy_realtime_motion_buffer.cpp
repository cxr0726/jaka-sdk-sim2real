// Exercise the ORIGINAL production receiver without sockets, ROS or hardware.
// Preinclude its dependencies before exposing private methods in this test TU;
// the production header/source and their class layout remain unchanged.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "RealtimeMotionBuffer.hpp"
#define private public
#include "LegacyRealtimeMotionBuffer.hpp"
#undef private

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace
{
    using RLController::LegacyRealtimeMotionBuffer;
    using json = nlohmann::json;
    constexpr int64_t ms = 1000000;
    constexpr std::array<int, 27> expected_map = {
        0,6,12,1,7,13,25,19,2,8,14,26,20,3,9,15,21,4,10,16,22,5,11,17,23,18,24};

    void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
    int64_t wall_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    json payload(const char *joint_key = "joint_pos", bool prefix = false)
    {
        std::vector<float> joints;
        if (prefix) joints.assign(7, -100.f);
        for (int j = 0; j < 27; ++j) joints.push_back(j + .125f);
        std::vector<std::vector<float>> pos(14, {4.f,5.f,9.f});
        std::vector<std::vector<float>> quat(14, {0.f,0.f,0.f,1.f});
        pos[0] = {1.f,2.f,.617f}; quat[0] = {2.f,0.f,0.f,0.f};
        return {{joint_key,joints},{"body_pos_w",pos},{"body_quat_w",quat},
                {"publish_t_ns",1},{"source_t_ns",9000000000000000000LL},
                {"stream_id","ignored-source-session"},{"seq",123}};
    }
}

int main()
{
    try
    {
        const std::vector<std::string> names(27, "joint");
        Eigen::VectorXf default_q = Eigen::VectorXf::Constant(27, -.2f);
        LegacyRealtimeMotionBuffer buffer(names, "", 1, 20*ms, 40*ms, {0,1,2,3,4},
                                    default_q, {0.f,0.f,.65f}, Eigen::Quaternionf::Identity());
        require(!buffer._zmq_socket && !buffer._zmq_context && !buffer._zmq_thread.joinable(),
                "empty endpoint created a network receiver");
        require(!buffer.ready() && buffer.latest_timestamp_ns() == 0, "empty legacy buffer became ready");
        require(buffer._delay_ns == 120*ms && buffer._history_ns == 120*ms,
                "legacy delay must equal max_future*dt+tolerance");
        for (const auto &frame : buffer.get_obs())
            require(frame.root_pos_w.z() == .65f && frame.dof_pos == default_q,
                    "empty legacy buffer default reference changed");

        auto packet = payload();
        const auto before = wall_ns(); buffer.append_payload(packet.dump()); const auto after = wall_ns();
        require(buffer.ready() && buffer._timestamps_ns.size() == 1,
                "first valid packet must be ready immediately without precharge");
        require(buffer.latest_timestamp_ns() >= before && buffer.latest_timestamp_ns() <= after,
                "legacy insert did not use local system_clock");
        for (const auto &frame : buffer.get_obs())
        {
            require(frame.root_pos_w == Eigen::Vector3f(1.f,2.f,.617f),
                    "legacy root must use body[0], with no extra height offset");
            require(frame.root_quat_w.w() == 1.f && frame.root_quat_w.vec().norm() == 0.f,
                    "legacy root quaternion WXYZ/normalization changed");
            for (int j = 0; j < 27; ++j)
                require(frame.dof_pos[j] == expected_map[j] + .125f,
                        "legacy MuJoCo-to-Isaac joint permutation changed");
        }
        // Missing, reversed, invalid-typed publication clocks and sequence IDs
        // are deliberately irrelevant to this original arrival-clock backend.
        packet["publish_t_ns"] = "ignored"; packet["seq"] = -1;
        buffer.append_payload(packet.dump());
        packet.erase("publish_t_ns"); packet.erase("source_t_ns"); packet.erase("stream_id");
        buffer.append_payload(packet.dump());
        require(buffer._timestamps_ns.size() == 3, "legacy buffer started interpreting publisher clock/session fields");
        for (const char *key : {"dof_pos","qpos"})
        {
            buffer.append_payload(payload(key, true).dump());
            for (int j = 0; j < 27; ++j)
                require(buffer._joint_pos_frames.back()[j] == expected_map[j] + .125f,
                        "legacy qpos/dof_pos prefix stripping changed");
        }
        const size_t valid_count = buffer._timestamps_ns.size();
        auto invalid = payload(); invalid["joint_pos"] = std::vector<float>(26, 0.f);
        bool rejected = false;
        try { buffer.append_payload(invalid.dump()); } catch (const std::runtime_error &) { rejected = true; }
        require(rejected && buffer._timestamps_ns.size() == valid_count,
                "invalid joint dimension changed stored reference frames");

        // Public get_obs must query LOCAL wall time -120ms +{0,20,40,60,80}ms.
        // A reference x(t)=(t-origin)/10ms exposes its exact sampling phase.
        const int64_t origin = wall_ns() - 200*ms;
        buffer._timestamps_ns = {origin, origin + 400*ms};
        buffer._root_pos_frames = {{0.f,0.f,.6f},{40.f,0.f,.6f}};
        buffer._root_quat_frames = {Eigen::Quaternionf::Identity(),Eigen::Quaternionf::Identity()};
        buffer._joint_pos_frames = {Eigen::VectorXf::Zero(27),Eigen::VectorXf::Constant(27,40.f)};
        const auto query_before = wall_ns(); const auto sampled = buffer.get_obs(); const auto query_after = wall_ns();
        for (int step = 0; step < 5; ++step)
        {
            const double low = double(query_before - 120*ms + step*20*ms - origin) / double(10*ms);
            const double high = double(query_after - 120*ms + step*20*ms - origin) / double(10*ms);
            require(sampled[step].root_pos_w.x() >= low-1.e-4 && sampled[step].root_pos_w.x() <= high+1.e-4,
                    "public get_obs used a different clock/window phase");
            require(std::abs(sampled[step].dof_pos[7] - sampled[step].root_pos_w.x()) < 1.e-5,
                    "root/joint interpolation timestamps diverged");
        }
        // Deterministic interpolation/endpoint clamping, including quaternion SLERP.
        buffer._timestamps_ns = {100,300};
        buffer._root_quat_frames = {
            Eigen::Quaternionf(Eigen::AngleAxisf(-.5f,Eigen::Vector3f::UnitZ())),
            Eigen::Quaternionf(Eigen::AngleAxisf(.5f,Eigen::Vector3f::UnitZ()))};
        std::array<RLController::InterpolatedFrame,5> exact;
        buffer.fill_sample_frames({0,100,200,300,400},exact);
        require(exact[0].root_pos_w.x() == 0.f && exact[4].root_pos_w.x() == 40.f &&
                exact[2].root_pos_w.x() == 20.f && std::abs(exact[2].root_quat_w.z()) < 1.e-6,
                "legacy interpolation/SLERP/endpoint clamp changed");
        // Original legacy receiver intentionally has no stale/readiness reset.
        buffer.get_obs();
        require(buffer.ready() && buffer._timestamps_ns.size() == 1 && buffer.get_obs()[0].root_pos_w.x() == 40.f,
                "legacy cleanup must retain/hold the final received frame");
        std::cout << "Actual legacy buffer: arrival clock, immediate ready, body[0]/no height offset, joint order, "
                     "qpos prefix, query phase, interpolation and terminal hold passed; no sockets/hardware\n";
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
