#ifndef __JK_STATE_PUBLISHER_H__
#define __JK_STATE_PUBLISHER_H__

#include <array>
#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

namespace RLController
{

    // ==========================================================================
    // StatePublisher — OpenHLM 风格的状态发布器(方案 A: 50Hz 控制 tick 内联)
    // ==========================================================================
    // 与 SIMPLE 侧 jaka_zmq_pub.py 的 state 流(:28711)保持一致(不含 qpos):
    //
    //   publish_t_ns : 发布时刻墙钟(ns)
    //   smplx_t_ns   : 恒 0(实机无仿真时间; recorder 端不读)
    //   paused       : 恒 false
    //   seq          : 单调递增
    //   joint_pos    : [27] JAKA 关节序(rad) — 调用方负责部署序→JAKA 序重排
    //   body_pos_w   : [28,3] 全零(recorder 端不读)
    //   body_quat_w  : [28,4] wxyz; 仅 [0] = base 四元数(imu_process 后)有意义,
    //                  其余 body 为 identity 四元数(recorder 端只读 [0])
    //
    // 发送侧防阻塞设计(对齐 OpenHLM ZMQOutputHandler):
    //   SNDHWM=1 + LINGER=0 + zmq_send(ZMQ_DONTWAIT) → 无订阅者/拥塞时直接丢帧,
    //   控制环零等待。
    // ==========================================================================
    class StatePublisher
    {
    public:
        explicit StatePublisher(const std::string &bind_addr = "tcp://*:28711");
        ~StatePublisher();

        StatePublisher(const StatePublisher &) = delete;
        StatePublisher &operator=(const StatePublisher &) = delete;

        /// 每个 50Hz 控制 tick 调用一次(内部非阻塞, 绝不阻塞控制环)。
        /// joint_pos_jaka : 27 维 JAKA 序关节位置(rad)
        /// base_quat_wxyz : 基座姿态四元数 wxyz(imu_process 后)
        void publish(const std::array<double, 27> &joint_pos_jaka,
                     const std::array<double, 4> &base_quat_wxyz);

        /// 初始化是否成功(bind 失败时为 false, publish() 空转, 不抛异常)
        bool ok() const { return _ok; }

    private:
        void *_zmq_context = nullptr;
        void *_zmq_socket = nullptr;
        bool _ok = false;
        uint64_t _seq = 0;
        nlohmann::json _body_pos_skeleton;  // [28,3] 全零, 构造时预构建
        nlohmann::json _body_quat_skeleton; // [28,4] identity, 第 0 行每 tick 覆盖
    };

} // namespace RLController

#endif // __JK_STATE_PUBLISHER_H__
