#include "RLController.hpp"
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

namespace RLController
{

    extern fsm_func_ptr StateJudge;
    extern fsm_func_ptr StateRun;
    extern FSMSTATE ReqState;

    static legged_waist_controller::RobotIntf::JoyRobotState joydata;
    static legged_waist_controller::RobotIntf::ImuData imu;
    static double q_cmd[50], q[50], qd[50];
    static double omega[3], acc[3], quat[4], grav_ori[3], Rot[3][3];
    static double pitch, roll, dpitch, droll, J[2][2];
    static Eigen::Quaternionf align_quat(1.0, 0.0, 0.0, 0.0);
    static int mimic_step;
    static bool mimic_play;

    static void from_fsm_mimic();

    static inline Eigen::Quaternionf z_angle_to_quat(double yaw)
    {
        const float half_yaw = static_cast<float>(0.5 * yaw);
        return Eigen::Quaternionf(std::cos(half_yaw), 0.0f, 0.0f, std::sin(half_yaw)).normalized();
    }

    static inline void construct_mimic_single_obs()
    {
        int i;
        const int frame = std::clamp(mimic_step, 0, mRefMotion.motion_frames - 1);

        AB2PitchRoll(q[4], q[5], qd[4], qd[5], &pitch, &roll, &dpitch, &droll, J);
        q[4] = pitch;
        q[5] = roll;
        qd[4] = dpitch;
        qd[5] = droll;
        AB2PitchRoll(-q[16], -q[17], -qd[16], -qd[17], &pitch, &roll, &dpitch, &droll, J);
        q[16] = pitch;
        q[17] = roll;
        qd[16] = dpitch;
        qd[17] = droll;

        if (mRobot->get_imu(imu))
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
            imu_process(acc, omega, quat);
            //new add
            q2R(quat, Rot);
            grav_ori[0] = -Rot[2][0];
            grav_ori[1] = -Rot[2][1];
            grav_ori[2] = -Rot[2][2];
        }
        else
        {
            omega[0] = omega[1] = omega[2] = 0.0;
            quat[0] = 1.0;
            quat[1] = quat[2] = quat[3] = 0.0;
        }

        if (mRefMotion.motion_frames > 0 && !mRefMotion.dof_pos.empty() && !mRefMotion.dof_vel.empty()
            && (int)mRefMotion.dof_pos[frame].size() >= mNumAction && (int)mRefMotion.dof_vel[frame].size() >= mNumAction)
        {
            for (i = 0; i < mNumAction; i++)
            {
                mSingleObsMimic[i] = static_cast<double>(mRefMotion.dof_pos[frame][i]);
                mSingleObsMimic[mNumAction + i] = static_cast<double>(mRefMotion.dof_vel[frame][i]);
            }

            // if (!mRefMotion.root_ori.empty() && mRefMotion.dof_pos[frame].size() > 24)
            // {
            //     const Eigen::Quaternionf rob_quat(
            //         static_cast<float>(quat[0]),
            //         static_cast<float>(quat[1]),
            //         static_cast<float>(quat[2]),
            //         static_cast<float>(quat[3]));

            //     const Eigen::Quaternionf ref_quat = mRefMotion.root_ori[frame] * z_angle_to_quat(mRefMotion.dof_pos[frame][24]);
            //     const Eigen::Matrix3f rot_diff = (rob_quat.conjugate() * (align_quat * ref_quat)).toRotationMatrix();
            //     mSingleObsMimic[2 * mNumAction + 0] = static_cast<double>(rot_diff(0, 0));
            //     mSingleObsMimic[2 * mNumAction + 1] = static_cast<double>(rot_diff(0, 1));
            //     mSingleObsMimic[2 * mNumAction + 2] = static_cast<double>(rot_diff(1, 0));
            //     mSingleObsMimic[2 * mNumAction + 3] = static_cast<double>(rot_diff(1, 1));
            //     mSingleObsMimic[2 * mNumAction + 4] = static_cast<double>(rot_diff(2, 0));
            //     mSingleObsMimic[2 * mNumAction + 5] = static_cast<double>(rot_diff(2, 1));
            // }
        }
        mSingleObsMimic[2 * mNumAction + 0] = grav_ori[0]*1.0;
        mSingleObsMimic[2 * mNumAction + 1] = grav_ori[1]*1.0;
        mSingleObsMimic[2 * mNumAction + 2] = grav_ori[2]*1.0;

        mSingleObsMimic[2 * mNumAction + 3] = omega[0] * mOmegaScale;
        mSingleObsMimic[2 * mNumAction + 4] = omega[1] * mOmegaScale;
        mSingleObsMimic[2 * mNumAction + 5] = omega[2] * mOmegaScale;

        for (i = 0; i < mNumAction; i++)
        {
            mSingleObsMimic[2 * mNumAction + 6 + i] = (q[i] - mDefaultPos[i]) * mDofPosScale;
            mSingleObsMimic[3 * mNumAction + 6 + i] = qd[i] * mDofVelScale;
            if (mMimicPolicy.output_buffer)
                mSingleObsMimic[4 * mNumAction + 6 + i] = static_cast<double>(mMimicPolicy.output_buffer[i]);
        }
        // mSingleObsMimic[2 * mNumAction + 6] = omega[0] * mOmegaScale;
        // mSingleObsMimic[2 * mNumAction + 7] = omega[1] * mOmegaScale;
        // mSingleObsMimic[2 * mNumAction + 8] = omega[2] * mOmegaScale;

        // for (i = 0; i < mNumAction; i++)
        // {
        //     mSingleObsMimic[2 * mNumAction + 9 + i] = (q[i] - mDefaultPos[i]) * mDofPosScale;
        //     mSingleObsMimic[3 * mNumAction + 9 + i] = qd[i] * mDofVelScale;
        //     if (mMimicPolicy.output_buffer)
        //         mSingleObsMimic[4 * mNumAction + 9 + i] = static_cast<double>(mMimicPolicy.output_buffer[i]);
        // }
    }

    static inline void push_mimic_obs_to_input_buffer()
    {
        int i, j;

        for (j = 0; j < mNumFrameStackMimic - 1; j++)
        {
            for (i = 0; i < mNumSingleObsMimic; i++)
            {
                mMimicPolicy.input_buffer[mNumSingleObsMimic * j + i] =
                    mMimicPolicy.input_buffer[mNumSingleObsMimic * (j + 1) + i];
            }
        }

        for (i = 0; i < mNumSingleObsMimic; i++)
        {
            mMimicPolicy.input_buffer[mNumSingleObsMimic * (mNumFrameStackMimic - 1) + i] =
                static_cast<float>(mSingleObsMimic[i]);
        }
    }

    static inline void fill_mimic_input_buffer()
    {
        int i, j;

        for (j = 0; j < mNumFrameStackMimic; j++)
        {
            for (i = 0; i < mNumSingleObsMimic; i++)
            {
                mMimicPolicy.input_buffer[mNumSingleObsMimic * j + i] =
                    static_cast<float>(mSingleObsMimic[i]);
            }
        }
    }

    static inline void init_mimic_policy_output_from_joint_cmd()
    {
        int i;
        if (!mMimicPolicy.output_buffer)
            return;

        if (mAngleMapFlag)
        {
            AB2PitchRoll(q_cmd[4], q_cmd[5], 0.0, 0.0, &pitch, &roll, &dpitch, &droll, J);
            q_cmd[4] = pitch;
            q_cmd[5] = roll;
            AB2PitchRoll(-q_cmd[16], -q_cmd[17], 0.0, 0.0, &pitch, &roll, &dpitch, &droll, J);
            q_cmd[16] = pitch;
            q_cmd[17] = roll;
        }

        for (i = 0; i < mNumAction; i++)
        {
            mMimicPolicy.output_buffer[i] = static_cast<float>((q_cmd[i] - mDefaultPos[i]) / mActionScale);
        }
    }

    static inline double clamp_mimic_joint_target(double target, int idx)
    {
        if (target > mUpLmt[idx])
        {
            return mUpLmt[idx];
        }
        if (target < mDownLmt[idx])
        {
            return mDownLmt[idx];
        }
        return target;
    }

    static inline void fill_mimic_trajectory_from_policy_output()
    {
        int i;
        if (!mMimicPolicy.output_buffer)
            return;

        for (i = 0; i < mNumAction; i++)
        {
            mPt.positions[i] =
                static_cast<double>(mMimicPolicy.output_buffer[i]) * mActionScale + mDefaultPos[i];
        }

        if (mAngleMapFlag)
        {
            double a, b;
            PitchRoll2AB(mPt.positions[4], mPt.positions[5], &a, &b);
            mPt.positions[4] = a;
            mPt.positions[5] = b;

            PitchRoll2AB(mPt.positions[16], mPt.positions[17], &a, &b);
            mPt.positions[16] = -a;
            mPt.positions[17] = -b;
        }

        for (i = 0; i < mNumAction; i++)
        {
            mPt.positions[i] = clamp_mimic_joint_target(mPt.positions[i], i);
        }

        mPt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(mTimeStep));
    }

    static void fsm_mimic_judge()
    {
        mRobot->get_joy_robot_state(joydata);
        if (mErrCode != ERR_NONE)
        {
            ReqState = FSM_FAULT;
        }
        else
        {
            if (joydata.joy.buttons_A)
            {
                mRobot->do_enable(false);
                ReqState = FSM_FAULT;
            }
            else if (joydata.joy.buttons_LB && joydata.joy.buttons_B)
            {
                ReqState = FSM_IDLE;
            }

            if (!mimic_play && joydata.joy.buttons_RB)
            {
                mimic_play = true;
            }
        }

        if (ReqState != FSM_MIMIC)
        {
            from_fsm_mimic();
        }
    }

    static void fsm_mimic_run()
    {
        update_joint_status(q_cmd, q, qd);
        if (mEnabledJointCount != mNumAction || mFaultJointCount > 0)
        {
            mErrCode = ERR_JOINT;
            return;
        }

        if (!mMimicPolicy.session)
        {
            mErrCode = ERR_INIT;
            return;
        }

        construct_mimic_single_obs();
        push_mimic_obs_to_input_buffer();

        mMimicPolicy.session->Run(
            Ort::RunOptions(nullptr),
            &mMimicPolicy.input_name,
            mMimicPolicy.input_tensor,
            1,
            &mMimicPolicy.output_name,
            mMimicPolicy.output_tensor,
            1);

        fill_mimic_trajectory_from_policy_output();
        mRobot->set_joint_trajectory(mJointNames, mPt);
        if (mimic_play)
        {
            if (mimic_step < mRefMotion.motion_frames)
                mimic_step++;
        }
    }

    void FSMMimicEnter(FSMSTATE from)
    {
        mFSMState = ReqState = FSM_MIMIC;
        StateJudge = &fsm_mimic_judge;
        StateRun = &fsm_mimic_run;
        mimic_step = 0;
        mimic_play = false;

        if (mDebugFlag)
            printf("--------from state %d to state %d------------\n", from, FSM_MIMIC);

        if (!mMimicPolicy.session || !mMimicPolicy.input_buffer || !mMimicPolicy.output_buffer)
        {
            mErrCode = ERR_INIT;
            if (mDebugFlag)
                printf("[FSMMimic] mimic policy not loaded, aborting\n");
            return;
        }

        update_joint_status(q_cmd, q, qd);
        init_mimic_policy_output_from_joint_cmd();
        construct_mimic_single_obs();
        fill_mimic_input_buffer();

        if (mRefMotion.motion_frames > 0 && !mRefMotion.root_ori.empty() && !mRefMotion.dof_pos.empty() && !mRefMotion.dof_pos[0].empty())
        {
            const Eigen::Quaternionf rob_quat(
                static_cast<float>(quat[0]),
                static_cast<float>(quat[1]),
                static_cast<float>(quat[2]),
                static_cast<float>(quat[3]));
            Eigen::Quaternionf rob_yaw0 = yawQuaternion(rob_quat);
            Eigen::Quaternionf ref_quat = mRefMotion.root_ori[0] * z_angle_to_quat(mRefMotion.dof_pos[0][24]);
            Eigen::Quaternionf ref_yaw0 = yawQuaternion(ref_quat);
            align_quat = rob_yaw0 * ref_yaw0.conjugate();
        }

        switch (from)
        {
        case FSM_IDLE:
            break;
        case FSM_JAKAMINIMIMIC_ZMQ:
            break;
        case FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST:
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }

    static void from_fsm_mimic()
    {
        switch (ReqState)
        {
        case FSM_IDLE:
            FSMIdleEnter(FSM_MIMIC);
            break;
        case FSM_FAULT:
            FSMFaultEnter(FSM_MIMIC);
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }
}
