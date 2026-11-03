#include "RLController.hpp"
#include "ParallelAnkleSolver.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace RLController
{
    namespace
    {
        // Owned by the controller thread; left/right never share a seed. SDK
        // substep validation uses a separate, deterministic stateless solve.
        std::array<jaka::ParallelAnkleSolver, 2> ankle_observers;
    }
    bool latch_ankle_fault(const char *context, const char *detail)
    {
        std::fprintf(stderr, "[ankle fault] %s: %s; disabling PD and latching ERR_JOINT\n",
                     context ? context : "unknown", detail ? detail : "invalid kinematics");
        mErrCode = ERR_JOINT;
        if (mRobot)
        {
            mRobot->enable_pd(false);
            mRobot->do_enable(false);
        }
        return false;
    }

    void reset_parallel_ankle_observers()
    {
        for (auto &observer : ankle_observers) observer.reset();
    }

    static bool decode_ankle_pair(double *q, double *qd, const char *context, bool observation)
    {
        if (!q || !qd || mNumAction < 18)
            return latch_ankle_fault(context, "invalid feedback dimensions");
        std::array<jaka::AnkleSolveResult, 2> result;
        for (int side = 0; side < 2; ++side)
        {
            const int i = side == 0 ? 4 : 16;
            const double sign = side == 0 ? 1.0 : -1.0;
            const auto canonical = jaka::solve_parallel_ankle(sign*q[i], sign*q[i+1], sign*qd[i], sign*qd[i+1]);
            result[side] = observation
                ? ankle_observers[side].solve(sign*q[i], sign*q[i+1], sign*qd[i], sign*qd[i+1]) : canonical;
            if (!canonical.ok() || !result[side].ok())
            {
                char detail[160];
                const auto status = !canonical.ok() ? canonical.status : result[side].status;
                std::snprintf(detail, sizeof(detail), "%s forward solve: %s", side == 0 ? "left" : "right",
                              jaka::ankle_status_name(status));
                return latch_ankle_fault(context, detail);
            }
            if (std::max(std::abs(result[side].pitch-canonical.pitch),
                         std::abs(result[side].roll-canonical.roll)) > 1.e-7)
                return latch_ankle_fault(context, "ambiguous assembly root between observer and SDK");
            // Share the same canonical measurement with the SDK's internal
            // stateless C call, after independently checking the warm-start root.
            result[side] = canonical;
        }
        for (int side = 0; side < 2; ++side)
        {
            const int i = side == 0 ? 4 : 16;
            q[i] = result[side].pitch; q[i+1] = result[side].roll;
            qd[i] = result[side].dpitch; qd[i+1] = result[side].droll;
        }
        return true;
    }

    bool decode_parallel_ankles(double *q, double *qd, const char *context)
    {
        return decode_ankle_pair(q, qd, context, true);
    }

    bool decode_parallel_ankle_commands(double *q_cmd, const char *context)
    {
        if (!q_cmd || mNumAction < 18 || mNumAction > 50)
            return latch_ankle_fault(context, "invalid command dimensions");
        std::array<double, 50> velocity{};
        return decode_ankle_pair(q_cmd, velocity.data(), context, false);
    }

    bool encode_parallel_ankle_targets(std::vector<double> &targets, const char *context)
    {
        if (targets.size() < 18)
            return latch_ankle_fault(context, "invalid target dimensions");
        double a[2], b[2];
        for (int side = 0; side < 2; ++side)
        {
            const int i = side == 0 ? 4 : 16;
            if (PitchRoll2ABChecked(targets[i], targets[i+1], &a[side], &b[side]) < 0)
                return latch_ankle_fault(context, side == 0 ? "left target is unreachable" : "right target is unreachable");
        }
        targets[4] = a[0]; targets[5] = b[0];
        targets[16] = -a[1]; targets[17] = -b[1];
        return true;
    }

    Eigen::Quaternionf yawQuaternion(const Eigen::Quaternionf &q)
    {
        float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
        float half_yaw = yaw * 0.5f;
        Eigen::Quaternionf ret(std::cos(half_yaw), 0.0f, 0.0f, std::sin(half_yaw));
        return ret.normalized();
    };

    static void qmlt(const double q1[4], const double q2[4], double ret[4])
    {
        const double w1 = q1[0];
        const double x1 = q1[1];
        const double y1 = q1[2];
        const double z1 = q1[3];

        const double w2 = q2[0];
        const double x2 = q2[1];
        const double y2 = q2[2];
        const double z2 = q2[3];

        ret[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        ret[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        ret[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        ret[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

        const double norm = sqrt(ret[0] * ret[0] + ret[1] * ret[1] + ret[2] * ret[2] + ret[3] * ret[3]);
        ret[0] /= norm;
        ret[1] /= norm;
        ret[2] /= norm;
        ret[3] /= norm;
        return;
    }

    // static void mvmlt(const double M[3][3], const double v[3], double ret[3])
    // {
    //     ret[0] = M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2];
    //     ret[1] = M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2];
    //     ret[2] = M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2];
    //     return;
    // }

    void q2R(const double q[4], double R[3][3])
    {
        const double w = q[0];
        const double x = q[1];
        const double y = q[2];
        const double z = q[3];

        R[0][0] = 1 - 2 * y * y - 2 * z * z;
        R[0][1] = 2 * x * y - 2 * w * z;
        R[0][2] = 2 * x * z + 2 * w * y;
        R[1][0] = 2 * x * y + 2 * w * z;
        R[1][1] = 1 - 2 * x * x - 2 * z * z;
        R[1][2] = 2 * y * z - 2 * w * x;
        R[2][0] = 2 * x * z - 2 * w * y;
        R[2][1] = 2 * y * z + 2 * w * x;
        R[2][2] = 1 - 2 * x * x - 2 * y * y;
        return;
    }

#define PI 3.14159265358979323846
    void imu_process(double acc[3], double omega[3], double quat[4])
    {
        double quat0[4] = {cos(-PI / 4), 0, 0, sin(-PI / 4)};
        double q[4];
        double tmp = acc[0];
        acc[0] = acc[1];
        acc[1] = -tmp;
        acc[2] = acc[2];

        tmp = omega[0];
        omega[0] = omega[1];
        omega[1] = -tmp;

        qmlt(quat0, quat, q);
        quat0[3] = -quat0[3];
        qmlt(q, quat0, quat);
    }

    void bind_nlf2_state()
    {
        if (mDataReadyFlag)
        {
            std::array<double, 50> q{}, qd{};
            if (mNumAction < 18 || mNumAction > 50)
            {
                latch_ankle_fault("initial planner binding", "invalid joint dimensions");
                return;
            }
            for (int i = 0; i < mNumAction; i++)
            {
                legged_waist_controller::RobotIntf::JointStatus jointstate{};
                if (mRobot->get_joint_status(mJointNames[i], jointstate) < 0 ||
                    !std::isfinite(jointstate.pos_fdb) || !std::isfinite(jointstate.vel_fdb))
                {
                    latch_ankle_fault("initial planner binding", "joint feedback read failed");
                    return;
                }
                q[i] = jointstate.pos_fdb; qd[i] = jointstate.vel_fdb;
            }
            if (!mAngleMapFlag && !decode_parallel_ankles(q.data(), qd.data(), "initial planner binding")) return;
            // Do not bind even one planner until all feedback is valid.
            for (int i = 0; i < mNumAction; ++i)
            {
                mNlf2[i].x0 = q[i]; mNlf2[i].v0 = qd[i];
            }
        }
    }

    void update_joint_status(double *q_cmd, double *q, double *qd)
    {
        if (mDataReadyFlag)
        {
            int i, enable_count = 0, fault_count = 0;
            for (i = 0; i < mNumAction; i++)
            {
                legged_waist_controller::RobotIntf::JointStatus joint_state{};
                if (mRobot->get_joint_status(mJointNames[i], joint_state) < 0 ||
                    !std::isfinite(joint_state.pos_fdb) || !std::isfinite(joint_state.vel_fdb) ||
                    !std::isfinite(joint_state.pos_cmd))
                {
                    ++fault_count;
                    q_cmd[i] = q[i] = qd[i] = std::numeric_limits<double>::quiet_NaN();
                    continue;
                }
                if (joint_state.is_enable)
                {
                    enable_count++;
                }
                if (joint_state.is_fault)
                {
                    fault_count++;
                }
                q_cmd[i] = joint_state.pos_cmd;
                q[i] = joint_state.pos_fdb;
                qd[i] = joint_state.vel_fdb;
            }
            mEnabledJointCount = enable_count;
            mFaultJointCount = fault_count;
        }
        else
        {
            int i;
            for (i = 0; i < mNumAction; i++)
            {
                q_cmd[i] = q[i] = qd[i] = 0.0;
            }
            mEnabledJointCount = 0;
            mFaultJointCount = 0;
        }
    }
}
