// ============================================================================
// StatePublisher.cpp — OpenHLM 风格状态发布器(50Hz 控制 tick 内联、非阻塞)
// ============================================================================
// 发布内容与 SIMPLE 侧 jaka_zmq_pub.py 的 state 流保持一致(不含 qpos):
//   publish_t_ns / smplx_t_ns(=0) / paused(=false) / seq /
//   joint_pos[27] JAKA 序 / body_pos_w[28,3] 全零 / body_quat_w[28,4](仅[0]有效)
// recorder 端(LatestStateClient)只消费 joint_pos + body_quat_w[0] + publish_t_ns。
// ============================================================================

#include "StatePublisher.hpp"

#include <zmq.h>

#include <chrono>
#include <cstdio>

namespace RLController
{

    namespace
    {
        inline int64_t wall_time_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }
    } // anonymous namespace

    StatePublisher::StatePublisher(const std::string &bind_addr)
    {
        _zmq_context = zmq_ctx_new();
        if (!_zmq_context)
        {
            std::fprintf(stderr, "[StatePublisher] zmq_ctx_new failed\n");
            return;
        }
        _zmq_socket = zmq_socket(_zmq_context, ZMQ_PUB);
        if (!_zmq_socket)
        {
            std::fprintf(stderr, "[StatePublisher] zmq_socket failed\n");
            return;
        }

        // 防阻塞: linger=0(关闭不等待), SNDHWM=1(只保留最新一条, 拥塞直接丢旧帧)
        int linger = 0;
        zmq_setsockopt(_zmq_socket, ZMQ_LINGER, &linger, sizeof(linger));
        int sndhwm = 1;
        zmq_setsockopt(_zmq_socket, ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));

        if (zmq_bind(_zmq_socket, bind_addr.c_str()) != 0)
        {
            std::fprintf(stderr, "[StatePublisher] bind %s failed: %s\n",
                         bind_addr.c_str(), zmq_strerror(zmq_errno()));
            return; // 绑定失败不崩溃: publish() 空转, 不影响控制环
        }

        // 预构建 28 body 骨架(body_pos 全零; body_quat identity, 第 0 行每 tick 覆盖)
        _body_pos_skeleton = nlohmann::json::array();
        _body_quat_skeleton = nlohmann::json::array();
        for (int i = 0; i < 28; ++i)
        {
            _body_pos_skeleton.push_back(nlohmann::json::array({0.0, 0.0, 0.0}));
            _body_quat_skeleton.push_back(nlohmann::json::array({1.0, 0.0, 0.0, 0.0}));
        }

        _ok = true;
        std::printf("[StatePublisher] bound to %s (state JSON, JAKA joint order)\n",
                    bind_addr.c_str());
    }

    StatePublisher::~StatePublisher()
    {
        if (_zmq_socket)
        {
            zmq_close(_zmq_socket);
            _zmq_socket = nullptr;
        }
        if (_zmq_context)
        {
            zmq_ctx_destroy(_zmq_context);
            _zmq_context = nullptr;
        }
    }

    void StatePublisher::publish(const std::array<double, 27> &joint_pos_jaka,
                                 const std::array<double, 4> &base_quat_wxyz)
    {
        if (!_ok)
            return;

        nlohmann::json body_quat = _body_quat_skeleton; // 拷贝 28 行骨架
        body_quat[0] = nlohmann::json::array(
            {base_quat_wxyz[0], base_quat_wxyz[1], base_quat_wxyz[2], base_quat_wxyz[3]});

        nlohmann::json payload;
        payload["publish_t_ns"] = wall_time_ns();
        payload["smplx_t_ns"] = 0; // 实机无仿真时间; recorder 端不读
        payload["paused"] = false;
        payload["seq"] = _seq++;
        payload["joint_pos"] = joint_pos_jaka;
        payload["body_pos_w"] = _body_pos_skeleton;
        payload["body_quat_w"] = std::move(body_quat);

        const std::string msg = payload.dump();
        // 非阻塞: 无订阅者/发送队列满时立即返回, 控制环零等待(OpenHLM dontwait 同款)
        zmq_send(_zmq_socket, msg.data(), msg.size(), ZMQ_DONTWAIT);
    }

} // namespace RLController
