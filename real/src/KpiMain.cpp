#include "RLController.hpp"
#include "rt_thread.hpp"
#include <stdio.h>
#include <signal.h>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>

static std::atomic<bool> main_loop{true};
static std::atomic<bool> data_log_thread_run{false};
static std::thread rl_fsm_thread;
static std::thread data_log_thread;
static constexpr size_t DATA_LOG_MAX_ROWS = 100000;

static void sigint_handler(int)
{
    main_loop = false;
    RLController::mRLThreadRunFlag = false;
    data_log_thread_run = false;
    printf("---------sigint catched-------------\n");
}

static std::string make_data_log_path()
{
    // const char *log_dir = KPI_LOG_BASE_DIR;
    const char *log_dir = "data_log";
    mkdir(log_dir, 0755);

    std::ostringstream path;
    path << log_dir << "/debug_data"
        << ".csv";
    return path.str();
}

static void write_data_log_header(std::ostream &fout)
{
    fout << "time_s,"
         << "imu_x,imu_y,imu_z,"
         << "imu_wx,imu_wy,imu_wz,"
         << "imu_ax,imu_ay,imu_az";

    for (const auto &joint_name : RLController::mJointNames)
    {
        fout << "," << joint_name << "_pos_cmd"
             << "," << joint_name << "_pos_fdb"
             << "," << joint_name << "_vel_fdb"
             << "," << joint_name << "_effort_fdb"
             << "," << joint_name << "_is_enable"
             << "," << joint_name << "_is_fault"
             << "," << joint_name << "_error_code"
             << "," << joint_name << "_ethercat_op";
    }
    fout << "\n";
}

static std::string make_fixed_width_data_log_row(std::string row, size_t row_slot_bytes, bool &row_truncated)
{
    if (row_slot_bytes == 0)
    {
        return row;
    }

    if (row.size() >= row_slot_bytes)
    {
        row.resize(row_slot_bytes - 1);
        row_truncated = true;
    }
    else
    {
        row.append(row_slot_bytes - row.size() - 1, ' ');
    }

    row.push_back('\n');
    return row;
}

static void data_log_rt_func()
{
    const std::chrono::milliseconds period(20);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(4, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    const std::string log_path = make_data_log_path();
    std::fstream fout(log_path, std::ios::in | std::ios::out | std::ios::trunc);
    if (!fout.is_open())
    {
        printf("!!!data log file open failed!!!\n");
        data_log_thread_run = false;
        return;
    }

    fout << std::fixed << std::setprecision(9);
    write_data_log_header(fout);
    const std::streamoff data_start_pos = fout.tellp();
    const size_t row_slot_bytes = 512 + RLController::mJointNames.size() * 256;
    size_t data_log_row_index = 0;
    bool data_log_wrapped = false;
    bool row_truncated = false;
    printf("data log thread started: %s\n", log_path.c_str());

    auto start_time = std::chrono::steady_clock::now();
    auto next_run = start_time;
    int over_time_count = 0;
    const int OVER_TIME_LOG_INTERVAL = 50;
    double acc[3], quat[4], omega[3], rpy[3];
    legged_waist_controller::RobotIntf::ImuData imu;

    while (data_log_thread_run && rclcpp::ok() && RLController::mLogData)
    {
        const auto sample_time = std::chrono::steady_clock::now();
        const double time_s = std::chrono::duration<double>(sample_time - start_time).count();

        // RobotIntf returns its latest thread-safe IMU snapshot; false means no sample yet.
        const bool imu_ok = RLController::mRobot && RLController::mRobot->get_imu(imu);
        if (imu_ok)
        {
            acc[0] = imu.ax;
            acc[1] = imu.ay;
            acc[2] = imu.az;
            omega[0] = imu.wx;
            omega[1] = imu.wy;
            omega[2] = imu.wz;
            quat[0] = imu.qw;
            quat[1] = imu.qx;
            quat[2] = imu.qy;
            quat[3] = imu.qz;
            RLController::imu_process(acc, omega, quat); // align frame from imu to the installed link
            Q2RPY(quat, rpy);
        }
        else
        {
            acc[0] = acc[1] = acc[2] = 0.0;
            omega[0] = omega[1] = omega[2] = 0.0;
            rpy[0] = rpy[1] = rpy[2] = 0.0;
        }

        std::ostringstream row_stream;
        row_stream << std::fixed << std::setprecision(9);
        row_stream << time_s
                   << "," << rpy[0]
                   << "," << rpy[1]
                   << "," << rpy[2]
                   << "," << omega[0]
                   << "," << omega[1]
                   << "," << omega[2]
                   << "," << acc[0]
                   << "," << acc[1]
                   << "," << acc[2];

        legged_waist_controller::RobotIntf::JointStatus joint_state;
        for (const auto &joint_name : RLController::mJointNames)
        {
            joint_state = legged_waist_controller::RobotIntf::JointStatus{};
            if (RLController::mRobot)
            {
                // Read RobotIntf's cached controller feedback for this log column.
                RLController::mRobot->get_joint_status(joint_name, joint_state);
            }

            row_stream << "," << joint_state.pos_cmd
                       << "," << joint_state.pos_fdb
                       << "," << joint_state.vel_fdb
                       << "," << joint_state.effort_fdb
                       << "," << static_cast<int>(joint_state.is_enable)
                       << "," << static_cast<int>(joint_state.is_fault)
                       << "," << joint_state.error_code
                       << "," << static_cast<int>(joint_state.ethercat_operational);
        }

        const size_t circular_row = data_log_row_index % DATA_LOG_MAX_ROWS;
        std::string row = make_fixed_width_data_log_row(row_stream.str(), row_slot_bytes, row_truncated);
        fout.seekp(data_start_pos + static_cast<std::streamoff>(circular_row * row_slot_bytes));
        fout.write(row.data(), static_cast<std::streamsize>(row.size()));
        if (!fout)
        {
            printf("!!!data log write failed!!!\n");
            data_log_thread_run = false;
            break;
        }

        data_log_row_index++;
        if (!data_log_wrapped && data_log_row_index == DATA_LOG_MAX_ROWS)
        {
            data_log_wrapped = true;
            printf("data log reached %zu rows; circular overwrite enabled\n", DATA_LOG_MAX_ROWS);
        }
        if (row_truncated)
        {
            printf("!!!data log row exceeded %zu bytes and was truncated!!!\n", row_slot_bytes);
            row_truncated = false;
        }

        next_run += period;
        const auto current_time = std::chrono::steady_clock::now();
        if (current_time < next_run)
        {
            std::this_thread::sleep_until(next_run);
        }
        else
        {
            over_time_count++;
            if (over_time_count % OVER_TIME_LOG_INTERVAL == 0)
            {
                auto over_time = std::chrono::duration_cast<std::chrono::microseconds>(current_time - next_run).count() / 1000.0;
                printf("data log over run for %d times, overtime: %.3f ms\n", over_time_count, over_time);
            }
            next_run = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    fout.flush();
    printf("data log thread stopped\n");
}

static void rt_func()
{
    const std::chrono::milliseconds period(20); // should be 20
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(5, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    auto next_run = std::chrono::steady_clock::now();
    int over_time_count = 0;
    const int OVER_TIME_LOG_INTERVAL = 1;
    while (RLController::mRLThreadRunFlag && rclcpp::ok())
    {
        RLController::FSMUpdate();

        auto current_time = std::chrono::steady_clock::now();
        next_run += period;
        if (current_time < next_run)
        {
            std::this_thread::sleep_until(next_run);
        }
        else
        {
            // 超时：统计并按需打印日志
            over_time_count++;
            if (over_time_count % OVER_TIME_LOG_INTERVAL == 0)
            {
                auto over_time = std::chrono::duration_cast<std::chrono::microseconds>(current_time - next_run).count() / 1000.0;
                printf("over run for %d times, overtime: %.3f ms\n", over_time_count, over_time);
            }
            next_run = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }
}

int main(int argc, char *argv[])
{
    // Default rclcpp SIGINT handler calls shutdown() immediately; RobotIntf cleanup below still needs a valid context.
    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options);

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    // RLController::KpiConfigInit(KPI_CONFIG_BASE_DIR "/KpiConfig.yaml");
    RLController::KpiConfigInit("./config/KpiConfig.yaml");
    printf("RLController (variables) has been initialized from config.yaml\n");

    legged_waist_controller::RobotIntf::Options opt;
    opt.enable_joy_sub = true;
    // Construct the sole RobotIntf bridge; defaults start its internal ROS spin thread.
    RLController::mRobot = std::make_unique<legged_waist_controller::RobotIntf>(opt);
    if (!opt.auto_start_spin)
        // Required only if automatic spinning is disabled in the options above.
        RLController::mRobot->start();


    RLController::FSMInitEnter(RLController::FSM_INIT);

    rt_thread::ThreadParams rl_fsm_thread_params;
    rl_fsm_thread_params.name = "rl_fsm_thread";
    rl_fsm_thread_params.sched_policy = SCHED_FIFO;
    rl_fsm_thread_params.priority = 75;

    bool ret = rt_thread::RtThreadHelper::create_thread(
        rl_fsm_thread,
        rl_fsm_thread_params,
        &rt_func);

    if (!ret || RLController::mErrCode != RLController::ERR_NONE)
    {
        main_loop = false;
        RLController::mRLThreadRunFlag = false;
        printf("!!!rl_fsm_thread creation failed!!!\n");
    }

    if (RLController::mLogData)
    {
        rt_thread::ThreadParams data_log_thread_params;
        data_log_thread_params.name = "data_log_thread";
        data_log_thread_params.sched_policy = SCHED_FIFO;
        data_log_thread_params.priority = 65;

        data_log_thread_run = true;
        bool log_ret = rt_thread::RtThreadHelper::create_thread(
            data_log_thread,
            data_log_thread_params,
            &data_log_rt_func);

        if (!log_ret)
        {
            data_log_thread_run = false;
            printf("!!!data_log_thread creation failed!!!\n");
        }
    }

    while (main_loop)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RLController::mRLThreadRunFlag = false;
    data_log_thread_run = false;
    if (rl_fsm_thread.joinable())
    {
        rl_fsm_thread.join();
        printf("rl_fsm_thread join\n");
    }
    if (data_log_thread.joinable())
    {
        data_log_thread.join();
        printf("data_log_thread join\n");
    }
    // Request a safe servo-off state before tearing down the ROS communication path.
    RLController::mRobot->do_enable(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RLController::KpiConfigDeinit();
    // Stop RobotIntf while rclcpp is still valid, then destroy it after ROS shutdown.
    RLController::mRobot->stop();
    rclcpp::shutdown();
    RLController::mRobot.reset();

    return 0;
}
