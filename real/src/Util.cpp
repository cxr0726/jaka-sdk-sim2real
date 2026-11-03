#include "RLController.hpp"

namespace RLController
{
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
            int i;
            legged_waist_controller::RobotIntf::JointStatus jointstate;
            for (i = 0; i < mNumAction; i++)
            {

                // Seed each local filter from RobotIntf's cached joint feedback.
                mRobot->get_joint_status(mJointNames[i], jointstate);
                mNlf2[i].x0 = jointstate.pos_fdb;
                mNlf2[i].v0 = jointstate.vel_fdb;
            }

            if (!mAngleMapFlag)
            {
                double pitch, roll, dpitch, droll, J[2][2];
                AB2PitchRoll(mNlf2[4].x0, mNlf2[5].x0, mNlf2[4].v0, mNlf2[5].v0, &pitch, &roll, &dpitch, &droll, J);
                mNlf2[4].x0 = pitch;
                mNlf2[5].x0 = roll;
                mNlf2[4].v0 = dpitch;
                mNlf2[5].v0 = droll;

                AB2PitchRoll(-mNlf2[16].x0, -mNlf2[17].x0, -mNlf2[16].v0, -mNlf2[17].v0, &pitch, &roll, &dpitch, &droll, J);
                mNlf2[16].x0 = pitch;
                mNlf2[17].x0 = roll;
                mNlf2[16].v0 = dpitch;
                mNlf2[17].v0 = droll;
            }
        }
    }

    void update_joint_status(double *q_cmd, double *q, double *qd)
    {
        if (mDataReadyFlag)
        {
            legged_waist_controller::RobotIntf::JointStatus joint_state;
            int i, enable_count = 0, fault_count = 0;
            for (i = 0; i < mNumAction; i++)
            {
                // Snapshot controller feedback used by every active FSM state.
                mRobot->get_joint_status(mJointNames[i], joint_state);
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
