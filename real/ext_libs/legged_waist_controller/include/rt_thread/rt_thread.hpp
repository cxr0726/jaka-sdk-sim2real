#ifndef RT_THREAD_HPP_
#define RT_THREAD_HPP_

#include <thread>
#include <string>
#include <pthread.h>
#include <sched.h>
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace rt_thread
{

    /**
     * @brief 实时线程属性配置参数
     */
    struct ThreadParams
    {
        std::string name;              // 线程名称，最长15字符
        int sched_policy = SCHED_FIFO; // 调度策略：SCHED_FIFO/SCHED_RR/SCHED_OTHER
        int priority = 50;             // 优先级（实时策略下1~99，值越大优先级越高）
    };

    /**
     * @brief 实时线程工具类，提供线程创建和属性配置功能
     */
    class RtThreadHelper
    {
    public:
        /**
         * @brief 创建并配置实时线程（模板函数实现）
         * @tparam Function 线程函数类型（支持普通函数、lambda、成员函数）
         * @tparam Args 线程函数参数类型
         * @param[out] thread 输出的线程对象
         * @param params 线程属性配置参数
         * @param f 线程函数
         * @param args 线程函数参数
         * @return 成功返回true，失败返回false
         */
        template <typename Function, typename... Args>
        static bool create_thread(
            std::thread &thread,
            const ThreadParams &params,
            Function &&f,
            Args &&...args)
        {
            try
            {
                // 创建线程（完美转发参数）
                thread = std::thread(
                    std::forward<Function>(f),
                    std::forward<Args>(args)...);
            }
            catch (const std::system_error &e)
            {
                std::cerr << "线程创建失败: " << e.what() << std::endl;
                return false;
            }

            // 配置线程属性（名称、调度策略、优先级）
            return set_thread_attributes(thread, params);
        }

        /**
         * @brief 为已创建的线程配置属性（名称、调度策略、优先级）
         * @param thread 已创建的线程对象（必须可joinable）
         * @param params 线程属性配置参数
         * @return 成功返回true，失败返回false
         */
        static bool set_thread_attributes(
            std::thread &thread,
            const ThreadParams &params)
        {
            if (!thread.joinable())
            {
                std::cerr << "线程不可joinable，无法配置属性" << std::endl;
                return false;
            }

            pthread_t pthread = thread.native_handle();

            // 1. 设置线程名称（限制15字符）
            if (!params.name.empty())
            {
                std::string truncated_name = params.name.substr(0, 15); // pthread名称最长15字节
                if (pthread_setname_np(pthread, truncated_name.c_str()) != 0)
                {
                    std::cerr << "设置线程名称失败: " << strerror(errno) << std::endl;
                    // 名称设置失败不影响线程功能，继续执行
                }
            }

            // 2. 检查调度策略合法性
            if (!is_valid_sched_policy(params.sched_policy))
            {
                std::cerr << "不支持的调度策略: " << params.sched_policy << std::endl;
                return false;
            }

            // 3. 检查并调整优先级
            int adjusted_prio;
            if (!check_priority_range(params.sched_policy, params.priority, adjusted_prio))
            {
                std::cerr << "优先级超出范围: " << params.priority << std::endl;
                return false;
            }

            // 4. 设置调度策略和优先级
            struct sched_param sched_params;
            std::memset(&sched_params, 0, sizeof(sched_params));
            sched_params.sched_priority = adjusted_prio;

            if (pthread_setschedparam(pthread, params.sched_policy, &sched_params) != 0)
            {
                std::cerr << "设置调度参数失败: " << strerror(errno)
                          << " (可能需要CAP_SYS_NICE权限)" << std::endl;
                return false;
            }

            return true;
        }

    private:
        // 检查调度策略合法性
        static bool is_valid_sched_policy(int policy)
        {
            return (policy == SCHED_FIFO || policy == SCHED_RR || policy == SCHED_OTHER);
        }

        // 检查优先级是否在合法范围并调整
        static bool check_priority_range(int policy, int priority, int &adjusted_prio)
        {
            int min_prio, max_prio;

            if (policy == SCHED_OTHER)
            {
                // 非实时策略优先级固定为0
                min_prio = max_prio = 0;
            }
            else
            {
                // 实时策略（FIFO/RR）优先级范围1~99
                min_prio = 1;
                max_prio = 99;
            }

            if (priority < min_prio || priority > max_prio)
            {
                adjusted_prio = std::clamp(priority, min_prio, max_prio);
                std::cerr << "优先级自动调整为合法范围: " << adjusted_prio << std::endl;
                return false; // 优先级越界，返回失败
            }

            adjusted_prio = priority;
            return true;
        }
    };

} // namespace rt_thread

#endif // RT_THREAD_HPP_
