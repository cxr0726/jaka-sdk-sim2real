#include "MujocoRobotBackend.hpp"
#include "CheckedMujocoPhysics.hpp"
#include "RLController.hpp"

#if KPI_MUJOCO_HAS_VIEWER
#include "MujocoViewer.hpp"
#include "ViewerSnapshot.hpp"
// The packaged SDK provides GLFW's binary and forward-declares GLFWwindow,
// but does not ship glfw3.h. These two stable C ABI functions need no new SDK.
extern "C" void glfwSwapInterval(int interval);
extern "C" GLFWwindow *glfwGetCurrentContext(void);
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace
{
std::atomic<bool> g_run{true};

void signal_handler(int)
{
    g_run = false;
}

struct Args
{
    std::string config_path;
    std::string xml_path;
    std::string mode{"idle"};
    std::string direct_fsm;
    std::string log_path;
    std::string timing_log_path;
    double duration{20.0};
    double simulation_dt{0.001};
    int control_decimation{20};
    bool realtime{false};
    bool viewer{false};
    bool viewer_vsync{false};
    double viewer_fps{60.0};
    double loco_forward{0.0};
    double loco_lateral{0.0};
    double loco_yaw{0.0};
    std::string joy_file;
    bool joy_file_start_loco{true};
    bool ros_joy{false};
    std::string ros_joy_topic{"/joy"};
    bool ros_joy_start_loco{true};
};

void print_usage(const char *argv0)
{
    std::cerr
        << "Usage: " << argv0 << " --config <KpiConfig.demo.yaml> --xml <Khan_mini_sym.xml> [options]\n"
        << "Options:\n"
        << "  --mode <idle|loco|mimic|jakaminimimic_zmq|jakaminimimic_zmq_fixwrist|jakaminimimic_superzmq>  default: idle\n"
        << "  --direct-fsm <idle|loco|mimic|jakaminimimic_zmq|jakaminimimic_zmq_fixwrist|jakaminimimic_superzmq>\n"
        << "                                      enter target FSM after INIT without scripted idle buttons\n"
        << "  --duration <seconds>                default: 20, <=0 means until Ctrl-C\n"
        << "  --simulation-dt <seconds>           default: 0.001\n"
        << "  --control-decimation <n>            default: 20\n"
        << "  --log <csv>                         optional MuJoCo feedback/target log\n"
        << "  --viewer                            open MuJoCo GLFW viewer\n"
        << "  --viewer-fps <fps>                  display rate, default: 60; control is independent\n"
        << "  --viewer-vsync                      enable display VSync (default: off)\n"
        << "  --timing-log <csv>                  control intervals, work and realtime lag\n"
        << "  --realtime                          sleep to approximate real time\n"
        << "  --loco-forward <value>              LOCO left-stick Y, default 0\n"
        << "  --loco-lateral <value>              LOCO right-stick X, default 0\n"
        << "  --loco-yaw <value>                  LOCO left-stick X, default 0\n"
        << "  --joy-file <path>                   read real/synthetic joystick state from text file\n"
        << "  --joy-file-no-start-loco            do not auto-bootstrap LOCO before reading joy file\n"
        << "  --ros-joy                           subscribe ROS sensor_msgs/Joy and feed full C++ MuJoCo sim\n"
        << "  --ros-joy-topic <topic>             ROS joy topic, default /joy\n"
        << "  --ros-joy-no-start-loco             do not auto-bootstrap LOCO before reading ROS joy\n";
}

bool parse_args(int argc, char **argv, Args &args)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string key = argv[i];
        auto need_value = [&](const char *name) -> const char * {
            if (i + 1 >= argc)
            {
                std::cerr << "missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (key == "--config")
        {
            const char *v = need_value("--config");
            if (!v) return false;
            args.config_path = v;
        }
        else if (key == "--xml")
        {
            const char *v = need_value("--xml");
            if (!v) return false;
            args.xml_path = v;
        }
        else if (key == "--mode")
        {
            const char *v = need_value("--mode");
            if (!v) return false;
            args.mode = v;
        }
        else if (key == "--direct-fsm")
        {
            const char *v = need_value("--direct-fsm");
            if (!v) return false;
            args.direct_fsm = v;
        }
        else if (key == "--duration")
        {
            const char *v = need_value("--duration");
            if (!v) return false;
            args.duration = std::atof(v);
        }
        else if (key == "--simulation-dt")
        {
            const char *v = need_value("--simulation-dt");
            if (!v) return false;
            args.simulation_dt = std::atof(v);
        }
        else if (key == "--control-decimation")
        {
            const char *v = need_value("--control-decimation");
            if (!v) return false;
            args.control_decimation = std::atoi(v);
        }
        else if (key == "--log")
        {
            const char *v = need_value("--log");
            if (!v) return false;
            args.log_path = v;
        }
        else if (key == "--timing-log")
        {
            const char *v = need_value("--timing-log");
            if (!v) return false;
            args.timing_log_path = v;
        }
        else if (key == "--viewer-fps")
        {
            const char *v = need_value("--viewer-fps");
            if (!v) return false;
            args.viewer_fps = std::atof(v);
        }
        else if (key == "--viewer-vsync")
        {
            args.viewer_vsync = true;
        }
        else if (key == "--loco-forward")
        {
            const char *v = need_value("--loco-forward");
            if (!v) return false;
            args.loco_forward = std::atof(v);
        }
        else if (key == "--loco-lateral")
        {
            const char *v = need_value("--loco-lateral");
            if (!v) return false;
            args.loco_lateral = std::atof(v);
        }
        else if (key == "--loco-yaw")
        {
            const char *v = need_value("--loco-yaw");
            if (!v) return false;
            args.loco_yaw = std::atof(v);
        }
        else if (key == "--joy-file")
        {
            const char *v = need_value("--joy-file");
            if (!v) return false;
            args.joy_file = v;
        }
        else if (key == "--joy-file-no-start-loco")
        {
            args.joy_file_start_loco = false;
        }
        else if (key == "--ros-joy")
        {
            args.ros_joy = true;
        }
        else if (key == "--ros-joy-topic")
        {
            const char *v = need_value("--ros-joy-topic");
            if (!v) return false;
            args.ros_joy_topic = v;
        }
        else if (key == "--ros-joy-no-start-loco")
        {
            args.ros_joy_start_loco = false;
        }
        else if (key == "--realtime")
        {
            args.realtime = true;
        }
        else if (key == "--viewer")
        {
            args.viewer = true;
            args.realtime = true;
        }
        else if (key == "--help" || key == "-h")
        {
            return false;
        }
        else
        {
            std::cerr << "unknown argument: " << key << "\n";
            return false;
        }
    }

    if (args.config_path.empty() || args.xml_path.empty())
    {
        return false;
    }
    if (args.mode != "idle" && args.mode != "loco" && args.mode != "mimic" &&
        args.mode != "jakaminimimic_zmq" && args.mode != "jakaminimimic_zmq_fixwrist" &&
        args.mode != "jakaminimimic_superzmq")
    {
        std::cerr << "invalid mode: " << args.mode << "\n";
        return false;
    }
    if (!args.direct_fsm.empty() &&
        args.direct_fsm != "idle" &&
        args.direct_fsm != "loco" &&
        args.direct_fsm != "mimic" &&
        args.direct_fsm != "jakaminimimic_zmq" &&
        args.direct_fsm != "jakaminimimic_zmq_fixwrist" &&
        args.direct_fsm != "jakaminimimic_superzmq")
    {
        std::cerr << "invalid direct FSM: " << args.direct_fsm << "\n";
        return false;
    }
    if (args.control_decimation <= 0 || args.simulation_dt <= 0.0)
    {
        std::cerr << "simulation dt and control decimation must be positive\n";
        return false;
    }
    if (!std::isfinite(args.viewer_fps) || args.viewer_fps < 1.0 || args.viewer_fps > 240.0)
    {
        std::cerr << "viewer fps must be between 1 and 240\n";
        return false;
    }
    const std::string selected_mode = args.direct_fsm.empty() ? args.mode : args.direct_fsm;
    if (selected_mode == "jakaminimimic_superzmq" &&
        (!std::isfinite(args.simulation_dt) ||
         std::abs(args.simulation_dt * args.control_decimation - 0.02) > 1.e-9))
    {
        std::cerr << "SuperMimic requires simulation-dt * control-decimation = 0.02 seconds\n";
        return false;
    }
    return true;
}

bool enter_direct_fsm_from_idle(const std::string &direct_fsm)
{
    if (direct_fsm.empty())
    {
        return true;
    }
    if (RLController::mFSMState != RLController::FSM_IDLE)
    {
        return false;
    }

    if (direct_fsm == "idle")
    {
        return true;
    }
    RLController::mRobot->do_enable(true);
    if (direct_fsm == "loco")
    {
        RLController::FSMLocoEnter(RLController::FSM_IDLE);
        return true;
    }
    if (direct_fsm == "mimic")
    {
        RLController::FSMMimicEnter(RLController::FSM_IDLE);
        return true;
    }
    if (direct_fsm == "jakaminimimic_zmq")
    {
        // ZMQ mimic must be entered only once every joint reports enabled.
        if (RLController::mEnabledJointCount != RLController::mNumAction)
        {
            return false;
        }
        RLController::FSMMimicJakaMiniZmqEnter(RLController::FSM_IDLE);
        return true;
    }
    if (direct_fsm == "jakaminimimic_zmq_fixwrist")
    {
        // ZMQ fixwrist mimic must be entered only once every joint reports enabled.
        if (RLController::mEnabledJointCount != RLController::mNumAction)
        {
            return false;
        }
        RLController::FSMMimicJakaMiniZmqFixwristEnter(RLController::FSM_IDLE);
        return true;
    }

    if (direct_fsm == "jakaminimimic_superzmq")
    {
        if (RLController::mEnabledJointCount != RLController::mNumAction)
            return false;
        RLController::FSMMimicJakaMiniSuperZmqEnter(RLController::FSM_IDLE);
        return true;
    }

    return false;
}

float joy_axis_at(const sensor_msgs::msg::Joy &msg, size_t index)
{
    return index < msg.axes.size() ? msg.axes[index] : 0.0f;
}

int joy_button_at(const sensor_msgs::msg::Joy &msg, size_t index)
{
    return index < msg.buttons.size() ? msg.buttons[index] : 0;
}

bool yaml_bool(const YAML::Node &node, const std::string &key, bool fallback)
{
    return node[key] ? node[key].as<bool>() : fallback;
}

legged_waist_controller::RobotIntf::JoyRobotState scripted_joy(
    const Args &args,
    int idle_ticks,
    int rb_pulses_sent,
    int control_tick,
    int mimic_ticks,
    double sim_time)
{
    legged_waist_controller::RobotIntf::JoyRobotState joy;
    joy.joy_online = true;

    switch (RLController::mFSMState)
    {
    case RLController::FSM_IDLE:
        if (idle_ticks < 2)
        {
            joy.joy.buttons_BACK = true;
        }
        else if (args.mode == "loco")
        {
            joy.joy.buttons_LB = true;
            joy.joy.buttons_Y = true;
        }
        else if (args.mode == "mimic")
        {
            joy.joy.buttons_LB = true;
            joy.joy.buttons_X = true;
        }
        else if (args.mode == "jakaminimimic_zmq")
        {
            // Wait for the enable request (BACK above) to take effect so the
            // ZMQ mimic FSM can fetch the joint state on entry.
            if (RLController::mEnabledJointCount == RLController::mNumAction)
            {
                joy.joy.buttons_LB = true;
                joy.joy.buttons_START = true;
            }
        }
        else if (args.mode == "jakaminimimic_zmq_fixwrist")
        {
            // Wait for the enable request (BACK above) to take effect so the
            // ZMQ fixwrist mimic FSM can fetch the joint state on entry.
            if (RLController::mEnabledJointCount == RLController::mNumAction)
            {
                joy.joy.buttons_LB = true;
                joy.joy.buttons_BACK = true;
            }
        }
        break;
    case RLController::FSM_LOCO:
        joy.joy.axes_L_Y = static_cast<float>(args.loco_forward);
        joy.joy.axes_R_X = static_cast<float>(args.loco_lateral);
        joy.joy.axes_L_X = static_cast<float>(args.loco_yaw);
        break;
    case RLController::FSM_MIMIC:
        if (mimic_ticks <= 1)
        {
            joy.joy.buttons_RB = true;
        }
        break;
    default:
        break;
    }

    return joy;
}

legged_waist_controller::RobotIntf::JoyRobotState neutral_joy()
{
    legged_waist_controller::RobotIntf::JoyRobotState joy;
    joy.joy_online = true;
    return joy;
}

legged_waist_controller::RobotIntf::JoyRobotState ros_msg_to_joy(const sensor_msgs::msg::Joy &msg)
{
    auto joy = neutral_joy();
    joy.joy_online = true;

    joy.joy.axes_L_X = joy_axis_at(msg, 0);
    joy.joy.axes_L_Y = joy_axis_at(msg, 1);
    joy.joy.axes_LT = joy_axis_at(msg, 2);
    joy.joy.axes_R_X = joy_axis_at(msg, 3);
    joy.joy.axes_R_Y = joy_axis_at(msg, 4);
    joy.joy.axes_RT = joy_axis_at(msg, 5);
    joy.joy.axes_DX = joy_axis_at(msg, 6);
    joy.joy.axes_DY = joy_axis_at(msg, 7);

    joy.joy.buttons_A = joy_button_at(msg, 0) != 0;
    joy.joy.buttons_B = joy_button_at(msg, 1) != 0;
    joy.joy.buttons_X = joy_button_at(msg, 2) != 0;
    joy.joy.buttons_Y = joy_button_at(msg, 3) != 0;
    joy.joy.buttons_LB = joy_button_at(msg, 4) != 0;
    joy.joy.buttons_RB = joy_button_at(msg, 5) != 0;
    joy.joy.buttons_BACK = joy_button_at(msg, 6) != 0;
    joy.joy.buttons_START = joy_button_at(msg, 7) != 0;
    return joy;
}

class RosJoyReader
{
public:
    RosJoyReader(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic)
        : node_(node)
    {
        sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            topic,
            rclcpp::QoS(10),
            [this](sensor_msgs::msg::Joy::SharedPtr msg) {
                if (!msg)
                {
                    return;
                }
                latest_ = ros_msg_to_joy(*msg);
                received_ = true;
            });
    }

    bool received() const { return received_; }

    legged_waist_controller::RobotIntf::JoyRobotState snapshot() const
    {
        if (received_)
        {
            return latest_;
        }
        auto joy = neutral_joy();
        joy.joy_online = false;
        return joy;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    legged_waist_controller::RobotIntf::JoyRobotState latest_{neutral_joy()};
    bool received_{false};
};

legged_waist_controller::RobotIntf::JoyRobotState file_joy(const std::string &path)
{
    auto joy = neutral_joy();
    std::ifstream in(path);
    if (!in)
    {
        return joy;
    }

    double lx = 0.0, ly = 0.0, rx = 0.0, ry = 0.0;
    double lt = 0.0, rt = 0.0, dx = 0.0, dy = 0.0;
    int a = 0, b = 0, x = 0, y = 0;
    int lb = 0, rb = 0, back = 0, start = 0, r3 = 0;
    if (!(in >> lx >> ly >> rx >> ry >> lt >> rt >> dx >> dy >>
          a >> b >> x >> y >> lb >> rb >> back >> start))
    {
        return joy;
    }
    in >> r3;

    joy.joy.axes_L_X = static_cast<float>(lx);
    joy.joy.axes_L_Y = static_cast<float>(ly);
    joy.joy.axes_R_X = static_cast<float>(rx);
    joy.joy.axes_R_Y = static_cast<float>(ry);
    joy.joy.axes_LT = static_cast<float>(lt);
    joy.joy.axes_RT = static_cast<float>(rt);
    joy.joy.axes_DX = static_cast<float>(dx);
    joy.joy.axes_DY = static_cast<float>(dy);
    joy.joy.buttons_A = a != 0;
    joy.joy.buttons_B = b != 0;
    joy.joy.buttons_X = x != 0;
    joy.joy.buttons_Y = y != 0;
    joy.joy.buttons_LB = lb != 0;
    joy.joy.buttons_RB = rb != 0;
    joy.joy.buttons_BACK = back != 0;
    joy.joy.buttons_START = start != 0;
    return joy;
}

void print_joy_if_changed(const legged_waist_controller::RobotIntf::JoyRobotState &joy)
{
    static bool initialized = false;
    static bool last_lb = false;
    static bool last_b = false;
    static bool last_rb = false;
    static bool last_y = false;
    static bool last_x = false;
    static bool last_a = false;
    static bool last_back = false;
    static double last_rt = -1.0;
    static double last_dx = -2.0;
    static double last_dy = -2.0;
    static int last_fsm = -1;
    const double rt = joy.joy.axes_RT;

    const bool changed =
        !initialized ||
        last_lb != joy.joy.buttons_LB ||
        last_b != joy.joy.buttons_B ||
        last_rb != joy.joy.buttons_RB ||
        last_y != joy.joy.buttons_Y ||
        last_x != joy.joy.buttons_X ||
        last_a != joy.joy.buttons_A ||
        last_back != joy.joy.buttons_BACK ||
        std::fabs(last_rt - rt) > 1.0e-3 ||
        std::fabs(last_dx - joy.joy.axes_DX) > 1.0e-3 ||
        std::fabs(last_dy - joy.joy.axes_DY) > 1.0e-3 ||
        last_fsm != static_cast<int>(RLController::mFSMState);

    if (changed)
    {
        initialized = true;
        last_lb = joy.joy.buttons_LB;
        last_b = joy.joy.buttons_B;
        last_rb = joy.joy.buttons_RB;
        last_y = joy.joy.buttons_Y;
        last_x = joy.joy.buttons_X;
        last_a = joy.joy.buttons_A;
        last_back = joy.joy.buttons_BACK;
        last_rt = rt;
        last_dx = joy.joy.axes_DX;
        last_dy = joy.joy.axes_DY;
        last_fsm = static_cast<int>(RLController::mFSMState);
        std::cout << "[kpi_mujoco_sim] joy"
                  << " fsm=" << static_cast<int>(RLController::mFSMState)
                  << " LB=" << joy.joy.buttons_LB
                  << " B=" << joy.joy.buttons_B
                  << " RB=" << joy.joy.buttons_RB
                  << " X=" << joy.joy.buttons_X
                  << " Y=" << joy.joy.buttons_Y
                  << " A=" << joy.joy.buttons_A
                  << " BACK=" << joy.joy.buttons_BACK
                  << " RT=" << rt
                  << " DX=" << joy.joy.axes_DX
                  << " DY=" << joy.joy.axes_DY
                  << " LX=" << joy.joy.axes_L_X
                  << " LY=" << joy.joy.axes_L_Y
                  << " RX=" << joy.joy.axes_R_X
                  << std::endl;
    }
}
} // namespace

int main(int argc, char **argv)
{
    Args args;
    if (!parse_args(argc, argv, args))
    {
        print_usage(argv[0]);
        return 2;
    }

    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try
    {
        RLController::KpiConfigInit(args.config_path.c_str());

        const std::string selected_mode = args.direct_fsm.empty() ? args.mode : args.direct_fsm;
        const bool super_mode = selected_mode == "jakaminimimic_superzmq";
        if (super_mode)
        {
            if (!RLController::mJakaMiniSuperZmqEnabled || !RLController::mJakaMiniSuperDeployment)
                throw std::runtime_error("SuperMimic mode requires a valid jakaminimimic_superzmq configuration");
            const auto &deployment = *RLController::mJakaMiniSuperDeployment;
            for (int policy = 0; policy < 27; ++policy)
            {
                const int deploy = deployment.policy_to_deploy[policy];
                RLController::mDefaultPos[deploy] = deployment.default_joint_pos[policy];
                RLController::mInitTarget[deploy] = deployment.default_joint_pos[policy];
                RLController::mKp[deploy] = deployment.joint_stiffness[policy];
                RLController::mKd[deploy] = deployment.joint_damping[policy];
            }
            // Metadata angles are physical ankle pitch/roll. The SDK's parallel
            // ankle interface uses the same actuator convention as existing modes.
            const int ankle_deploy[4] = {4, 5, 16, 17};
            for (int i = 0; i < 4; ++i)
            {
                RLController::mKp_PR[i] = RLController::mKp[ankle_deploy[i]];
                RLController::mKd_PR[i] = RLController::mKd[ankle_deploy[i]];
            }
            if (RLController::mAngleMapFlag)
            {
                double a, b;
                ::PitchRoll2AB(RLController::mInitTarget[4], RLController::mInitTarget[5], &a, &b);
                RLController::mInitTarget[4] = a;
                RLController::mInitTarget[5] = b;
                ::PitchRoll2AB(RLController::mInitTarget[16], RLController::mInitTarget[17], &a, &b);
                RLController::mInitTarget[16] = -a;
                RLController::mInitTarget[17] = -b;
            }
            RLController::mTimeStep = 0.02;
            std::cout << "[SuperMimic] default joints, initialization and PD gains loaded from ONNX metadata" << std::endl;
        }

        MujocoRobotBackend::Config sim_cfg;
        sim_cfg.xml_path = args.xml_path;
        sim_cfg.joint_names = RLController::mJointNames;
        sim_cfg.init_target = RLController::mInitTarget;
        sim_cfg.kp = RLController::mKp;
        sim_cfg.kd = RLController::mKd;
        sim_cfg.kp_pitch_roll = RLController::mKp_PR;
        sim_cfg.kd_pitch_roll = RLController::mKd_PR;
        sim_cfg.angle_map = RLController::mAngleMapFlag;
        sim_cfg.simulation_dt = args.simulation_dt;
        sim_cfg.control_decimation = args.control_decimation;
        sim_cfg.log_path = args.log_path;
        YAML::Node sim_yaml = YAML::LoadFile(args.config_path);
        if (sim_yaml["init_base_pos"])
        {
            sim_cfg.init_base_pos = sim_yaml["init_base_pos"].as<std::vector<double>>();
        }
        if (sim_yaml["init_base_quat"])
        {
            sim_cfg.init_base_quat = sim_yaml["init_base_quat"].as<std::vector<double>>();
        }
        if (sim_yaml["mujoco_imu_body_name"])
        {
            sim_cfg.imu_body_name = sim_yaml["mujoco_imu_body_name"].as<std::string>();
        }
        if (sim_yaml["mujoco_use_imu_body"])
        {
            sim_cfg.use_imu_body = yaml_bool(sim_yaml, "mujoco_use_imu_body", false);
        }
        // ZMQ mimic runs use the base/root IMU (like KpiDeployReal), not the
        // waist_yaw body used by the other mimic modes. Scoped per run because
        // the SDK backend has a single global IMU-body selection.
        const bool zmq_mode = args.mode == "jakaminimimic_zmq" ||
                              args.mode == "jakaminimimic_zmq_fixwrist" ||
                              args.direct_fsm == "jakaminimimic_zmq" ||
                              args.direct_fsm == "jakaminimimic_zmq_fixwrist";
        const bool zmq_fixwrist_mode = args.mode == "jakaminimimic_zmq_fixwrist" ||
                                       args.direct_fsm == "jakaminimimic_zmq_fixwrist";
        if (zmq_mode && sim_yaml["jakaminimimic_zmq"])
        {
            YAML::Node zc = sim_yaml["jakaminimimic_zmq"];
            sim_cfg.imu_body_name = zc["imu_body_name"]
                ? zc["imu_body_name"].as<std::string>()
                : sim_cfg.root_body_name;
            sim_cfg.use_imu_body = zc["use_imu_body"]
                ? yaml_bool(zc, "use_imu_body", false)
                : false;
        }
        if (zmq_fixwrist_mode && sim_yaml["jakaminimimic_zmq_fixwrist"])
        {
            YAML::Node zc = sim_yaml["jakaminimimic_zmq_fixwrist"];
            sim_cfg.imu_body_name = zc["imu_body_name"]
                ? zc["imu_body_name"].as<std::string>()
                : sim_cfg.root_body_name;
            sim_cfg.use_imu_body = zc["use_imu_body"]
                ? yaml_bool(zc, "use_imu_body", false)
                : false;
        }
        if (super_mode)
        {
            // The exported policy expects base/root IMU data. Reject an
            // accidental waist-body override instead of silently changing obs.
            const YAML::Node cfg = sim_yaml["jakaminimimic_superzmq"];
            if ((cfg["imu_body_name"] && cfg["imu_body_name"].as<std::string>() != sim_cfg.root_body_name) ||
                yaml_bool(cfg, "use_imu_body", false))
                throw std::runtime_error("SuperMimic requires base/root IMU: imu_body_name=base_link, use_imu_body=false");
            sim_cfg.imu_body_name = sim_cfg.root_body_name;
            sim_cfg.use_imu_body = false;
        }
        std::cout << "[kpi_mujoco_sim] imu source="
                  << (sim_cfg.use_imu_body ? sim_cfg.imu_body_name : sim_cfg.root_body_name)
                  << (sim_cfg.use_imu_body ? " body" : " root")
                  << std::endl;

        auto backend = std::make_shared<MujocoRobotBackend>(sim_cfg);
        jaka::CheckedMujocoPhysics checked_physics(*backend);
#if KPI_MUJOCO_HAS_VIEWER
        std::unique_ptr<jaka::ViewerSnapshot> viewer_snapshot;
        MujocoViewer viewer;
        if (args.viewer)
        {
            viewer_snapshot = std::make_unique<jaka::ViewerSnapshot>(backend->model(), backend->data());
            if (!viewer.open(viewer_snapshot->model(), viewer_snapshot->data(), "KpiDeployReal MuJoCo C++ Sim"))
                throw std::runtime_error("failed to open viewer");
            if (!glfwGetCurrentContext()) throw std::runtime_error("viewer has no current OpenGL context");
            glfwSwapInterval(args.viewer_vsync ? 1 : 0);
            std::cout << "[kpi_mujoco_sim] independent viewer fps=" << args.viewer_fps
                      << " vsync=" << args.viewer_vsync << "; physics runs on control thread" << std::endl;
        }
#else
        if (args.viewer)
        {
            std::cerr << "[kpi_mujoco_sim] viewer was not built. Reconfigure with KPI_MUJOCO_ENABLE_VIEWER=ON." << std::endl;
            return 1;
        }
#endif
        legged_waist_controller::RobotIntf::Options opt;
        opt.enable_joy_sub = true;
        RLController::mRobot = std::make_unique<legged_waist_controller::RobotIntf>(opt);
        RLController::mRobot->attach_backend(backend);

        RLController::FSMInitEnter(RLController::FSM_INIT);
        std::cout << "[kpi_mujoco_sim] started mode=" << args.mode
                  << " duration=" << args.duration
                  << " xml=" << args.xml_path << std::endl;

        int control_tick = 0;
        int idle_ticks = 0;
        int mimic_ticks = 0;
        // SuperMimic uses the existing direct-entry readiness path after
        // enabling, keeping START/BACK assignments in older modes unchanged.
        const std::string startup_fsm = args.direct_fsm.empty() && super_mode
            ? "jakaminimimic_superzmq" : args.direct_fsm;
        bool direct_fsm_done = startup_fsm.empty();
        const bool external_joy = !args.joy_file.empty() || args.ros_joy;
        const bool external_start_loco =
            startup_fsm.empty() &&
            ((!args.joy_file.empty() && args.joy_file_start_loco) ||
             (args.ros_joy && args.ros_joy_start_loco));
        bool external_joy_bootstrap_done = !external_joy || !external_start_loco;
        std::shared_ptr<rclcpp::Node> ros_joy_node;
        std::shared_ptr<RosJoyReader> ros_joy_reader;
        if (args.ros_joy)
        {
            ros_joy_node = std::make_shared<rclcpp::Node>("kpi_mujoco_ros_joy_reader");
            ros_joy_reader = std::make_shared<RosJoyReader>(ros_joy_node, args.ros_joy_topic);
            std::cout << "[kpi_mujoco_sim] ROS joy enabled: topic=" << args.ros_joy_topic << std::endl;
            if (args.ros_joy_start_loco)
            {
                std::cout << "[kpi_mujoco_sim] ROS joy bootstraps to LOCO first, then uses real ROS joy input" << std::endl;
            }
        }
        std::ofstream timing_log;
        if (!args.timing_log_path.empty())
        {
            timing_log.open(args.timing_log_path);
            if (!timing_log) throw std::runtime_error("cannot open control timing log");
            timing_log.precision(12);
            timing_log << "tick,wall_elapsed_s,sim_time_s,period_ms,work_before_log_ms,lag_before_log_ms\n";
        }
        const auto control_loop = [&]()
        {
        const auto wall_start = std::chrono::steady_clock::now();
        auto previous_start = wall_start;
        auto last_lag_warning = wall_start;
        while (g_run && RLController::mErrCode == RLController::ERR_NONE)
        {
            const auto tick_start = std::chrono::steady_clock::now();
            const double interval_ms = control_tick == 0 ? 0.0 :
                std::chrono::duration<double, std::milli>(tick_start - previous_start).count();
            previous_start = tick_start;
            if (args.duration > 0.0 && backend->simulation_time() >= args.duration)
            {
                break;
            }

            if (RLController::mFSMState == RLController::FSM_IDLE)
            {
                idle_ticks++;
            }
            if (RLController::mFSMState == RLController::FSM_MIMIC)
            {
                mimic_ticks++;
            }
            if (!direct_fsm_done && enter_direct_fsm_from_idle(startup_fsm))
            {
                direct_fsm_done = true;
                std::cout << "[kpi_mujoco_sim] direct FSM entered: " << startup_fsm << std::endl;
            }
            if (!external_joy_bootstrap_done && RLController::mFSMState == RLController::FSM_LOCO)
            {
                external_joy_bootstrap_done = true;
                std::cout << "[kpi_mujoco_sim] external joy bootstrap reached LOCO, using real joystick input" << std::endl;
            }

            legged_waist_controller::RobotIntf::JoyRobotState joy;
            if (ros_joy_node)
            {
                rclcpp::spin_some(ros_joy_node);
            }
            if (external_joy && !external_joy_bootstrap_done)
            {
                Args bootstrap_args = args;
                bootstrap_args.mode = "loco";
                joy = scripted_joy(bootstrap_args, idle_ticks, 0, control_tick, mimic_ticks, backend->simulation_time());
            }
            else if (args.ros_joy)
            {
                joy = ros_joy_reader ? ros_joy_reader->snapshot() : neutral_joy();
            }
            else if (args.joy_file.empty())
            {
                joy = scripted_joy(args, idle_ticks, 0, control_tick, mimic_ticks, backend->simulation_time());
            }
            else
            {
                joy = file_joy(args.joy_file);
            }
            if (external_joy)
            {
                print_joy_if_changed(joy);
            }
            backend->set_joy_state(joy);
            RLController::FSMUpdate();
            backend->set_fsm_state(static_cast<int>(RLController::mFSMState));

            // A failed observation or inverse target must never enter even one
            // more SDK PD substep with a fabricated/stale ankle state.
            if (RLController::mErrCode != RLController::ERR_NONE)
            {
                checked_physics.shutdown();
                if (RLController::mFSMState != RLController::FSM_FAULT)
                    RLController::FSMFaultEnter(RLController::mFSMState);
                break;
            }

            for (int i = 0; i < args.control_decimation; ++i)
            {
                if (!checked_physics.step())
                {
                    RLController::latch_ankle_fault("MuJoCo physics substep", checked_physics.error().c_str());
                    if (RLController::mFSMState != RLController::FSM_FAULT)
                        RLController::FSMFaultEnter(RLController::mFSMState);
                    break;
                }
            }
            if (RLController::mErrCode != RLController::ERR_NONE) break;

            if (control_tick % 50 == 0)
            {
                std::cout << "[kpi_mujoco_sim] t=" << backend->simulation_time()
                          << " state=" << static_cast<int>(RLController::mFSMState)
                          << " err=" << static_cast<int>(RLController::mErrCode) << std::endl;
            }

#if KPI_MUJOCO_HAS_VIEWER
            if (viewer_snapshot) viewer_snapshot->publish(backend->data());
#endif

            const auto work_end = std::chrono::steady_clock::now();
            const auto target_wall = wall_start + std::chrono::duration<double>(backend->simulation_time());
            const double lag_ms = std::max(0.0,
                std::chrono::duration<double, std::milli>(work_end - target_wall).count());
            if (timing_log)
                timing_log << control_tick << ','
                    << std::chrono::duration<double>(tick_start - wall_start).count() << ','
                    << backend->simulation_time() << ',' << interval_ms << ','
                    << std::chrono::duration<double, std::milli>(work_end - tick_start).count() << ','
                    << lag_ms << '\n';
            if (args.realtime && lag_ms > 1000.0 * RLController::mTimeStep &&
                work_end - last_lag_warning > std::chrono::seconds(1))
            {
                std::cerr << "[kpi_mujoco_sim] realtime control lag=" << lag_ms
                          << " ms; reference still advances with wall clock" << std::endl;
                last_lag_warning = work_end;
            }
            if (args.realtime)
            {
                std::this_thread::sleep_until(std::chrono::time_point_cast<std::chrono::steady_clock::duration>(target_wall));
            }
            control_tick++;
        }
        };

#if KPI_MUJOCO_HAS_VIEWER
        if (args.viewer)
        {
            std::atomic<bool> control_done{false};
            std::exception_ptr control_error;
            std::thread control([&]()
            {
                try { control_loop(); }
                catch (...) { control_error = std::current_exception(); }
                control_done.store(true, std::memory_order_release);
            });
            try
            {
                const auto display_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1.0 / args.viewer_fps));
                auto next_display = std::chrono::steady_clock::now();
                while (g_run && !control_done.load(std::memory_order_acquire) && viewer.is_open())
                {
                    viewer_snapshot->refresh();
                    viewer.render(viewer_snapshot->model(), viewer_snapshot->data());
                    next_display += display_period;
                    const auto now = std::chrono::steady_clock::now();
                    if (next_display < now) next_display = now;
                    std::this_thread::sleep_until(next_display);
                }
            }
            catch (...)
            {
                g_run = false;
                control.join();
                throw;
            }
            g_run = false;
            control.join();
            if (control_error) std::rethrow_exception(control_error);
        }
        else
#endif
        {
            control_loop();
        }

        std::cout << "[kpi_mujoco_sim] stopped t=" << backend->simulation_time()
                  << " state=" << static_cast<int>(RLController::mFSMState)
                  << " err=" << static_cast<int>(RLController::mErrCode) << std::endl;

        RLController::mRLThreadRunFlag = false;
        if (RLController::mRobot)
        {
            RLController::mRobot->enable_pd(false);
            RLController::mRobot->do_enable(false);
            RLController::mRobot->stop();
        }
        RLController::KpiConfigDeinit();
        RLController::mRobot.reset();
        rclcpp::shutdown();
    }
    catch (...)
    {
        try { throw; }
        catch (const std::exception &ex)
        {
            std::cerr << "[kpi_mujoco_sim] fatal: " << ex.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "[kpi_mujoco_sim] fatal: unknown control/viewer exception" << std::endl;
        }
        RLController::mRLThreadRunFlag = false;
        if (RLController::mRobot)
        {
            RLController::mRobot->enable_pd(false);
            RLController::mRobot->do_enable(false);
            RLController::mRobot->stop();
            RLController::mRobot.reset();
        }
        rclcpp::shutdown();
        return 1;
    }

    return RLController::mErrCode == RLController::ERR_NONE ? 0 : 1;
}
