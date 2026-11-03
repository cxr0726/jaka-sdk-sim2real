#include "RLController.hpp"
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace RLController
{

    extern fsm_func_ptr StateJudge;
    extern fsm_func_ptr StateRun;
    extern FSMSTATE ReqState;

    static legged_waist_controller::RobotIntf::LocoMoveCmd loco_cmd;
    static legged_waist_controller::RobotIntf::JoyRobotState joydata;
    static legged_waist_controller::RobotIntf::ImuData imu;
    static double q_cmd[50], q[50], qd[50];
    static double omega[3], acc[3], quat[4], grav_ori[3], Rot[3][3];
    static double pitch, roll, dpitch, droll, J[2][2];
    static legged_waist_controller::RobotIntf::ControlSource ctrl_source;

    static void from_fsm_loco();

    static inline void construct_loco_single_obs()
    {
        int i;

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

        // Pull RobotIntf's latest IMU snapshot for the policy observation.
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
            imu_process(acc, omega, quat); // align frame from imu to the installed link
            q2R(quat, Rot);
            grav_ori[0] = -Rot[2][0];
            grav_ori[1] = -Rot[2][1];
            grav_ori[2] = -Rot[2][2];
        }
        else
        {
            omega[0] = omega[1] = omega[2] = 0.0;
            grav_ori[0] = 0.0;
            grav_ori[1] = 0.0;
            grav_ori[2] = -1.0;
        }

        mSingleObs[0] = omega[0] * mOmegaScale;
        mSingleObs[1] = omega[1] * mOmegaScale;
        mSingleObs[2] = omega[2] * mOmegaScale;

        mSingleObs[3] = grav_ori[0];
        mSingleObs[4] = grav_ori[1];
        mSingleObs[5] = grav_ori[2];

        mSingleObs[6] = loco_cmd.forward * mCmdScale[0];
        if (mSingleObs[6] < -0.6)
            mSingleObs[6] = -0.6;
        mSingleObs[7] = loco_cmd.lateral * mCmdScale[1];
        mSingleObs[8] = loco_cmd.yaw * mCmdScale[2];

        for (i = 0; i < mNumAction; i++)
        {
            // qj
            mSingleObs[9 + i] = (q[i] - mDefaultPos[i]) * mDofPosScale;
            // dqj
            mSingleObs[9 + mNumAction + i] = qd[i] * mDofVelScale;
            // last action
            mSingleObs[9 + mNumAction * 2 + i] = static_cast<double>(mLocoPolicy.output_buffer[i]);
        }
    }

    static inline void push_loco_obs_to_input_buffer()
    {
        int i, j;

        for (j = 0; j < mNumFrameStack - 1; j++)
        {
            for (i = 0; i < mNumSingleObs; i++)
            {
                mLocoPolicy.input_buffer[mNumSingleObs * j + i] =
                    mLocoPolicy.input_buffer[mNumSingleObs * (j + 1) + i];
            }
        }

        for (i = 0; i < mNumSingleObs; i++)
        {
            mLocoPolicy.input_buffer[mNumSingleObs * (mNumFrameStack - 1) + i] =
                static_cast<float>(mSingleObs[i]);
        }
    }

    static inline void fill_loco_input_buffer()
    {
        int i, j;

        for (j = 0; j < mNumFrameStack; j++)
        {
            for (i = 0; i < mNumSingleObs; i++)
            {
                mLocoPolicy.input_buffer[mNumSingleObs * j + i] =
                    static_cast<float>(mSingleObs[i]);
            }
        }
    }

    static inline void init_loco_policy_output_from_joint_cmd()
    {
        int i;

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
            mLocoPolicy.output_buffer[i] = static_cast<float>((q_cmd[i] - mDefaultPos[i]) / mActionScale);
        }
    }

    static inline double clamp_loco_joint_target(double target, int idx)
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

    static inline void fill_loco_trajectory_from_policy_output()
    {
        int i;

        for (i = 0; i < mNumAction; i++)
        {
            double target = static_cast<double>(mLocoPolicy.output_buffer[i]) * mActionScale + mDefaultPos[i];
            mPt.positions[i] = target;
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
            mPt.positions[i] = clamp_loco_joint_target(mPt.positions[i], i);
        }

        mPt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(mTimeStep));
    }

    static void fsm_loco_judge()
    {
        // RobotIntf arbitrates input ownership and expires stale Joy samples.
        ctrl_source=mRobot->get_control_source();
        if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            mRobot->get_joy_robot_state(joydata);
        }


        if (mErrCode != ERR_NONE)
        {
            ReqState = FSM_FAULT;
        }
        else
        {
            if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
            {
                if (joydata.joy.buttons_A)
                {
                    mErrCode=ERR_ABORT;
                }
            }
        }

        if (ReqState != FSM_LOCO)
        {
            from_fsm_loco();
        }
    }

    static void fsm_loco_run()
    {
        update_joint_status(q_cmd, q, qd);
        if (mEnabledJointCount != mNumAction || mFaultJointCount > 0)
        {
            mErrCode = ERR_JOINT;
        }

        if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            // Refresh commanded velocity from the latest cached gamepad state.
            mRobot->get_joy_robot_state(joydata);
            loco_cmd.forward = static_cast<double>(joydata.joy.axes_L_Y);
            loco_cmd.yaw = static_cast<double>(joydata.joy.axes_L_X);
            loco_cmd.lateral = static_cast<double>(joydata.joy.axes_R_X);
        }

        construct_loco_single_obs();
        push_loco_obs_to_input_buffer();

        // run policy
        mLocoPolicy.session->Run(
            Ort::RunOptions(nullptr),
            &mLocoPolicy.input_name,
            mLocoPolicy.input_tensor,
            1,
            &mLocoPolicy.output_name,
            mLocoPolicy.output_tensor,
            1);


        fill_loco_trajectory_from_policy_output();
        // Send the policy target as an ordered single-point joint trajectory.
        mRobot->set_joint_trajectory(mJointNames, mPt);
    }

    void FSMLocoEnter(FSMSTATE from)
    {
        mFSMState = ReqState = FSM_LOCO;
        StateJudge = &fsm_loco_judge;
        StateRun = &fsm_loco_run;
        // Keep RobotIntf/PiState synchronized with the local FSM.
        mRobot->set_cur_control_stat(FSM_LOCO);
        joydata = legged_waist_controller::RobotIntf::JoyRobotState{};
        if (mDebugFlag)
            printf("--------from state %d to state %d------------\n", from, FSM_LOCO);

        loco_cmd.forward = 0.0;
        loco_cmd.lateral = 0.0;
        loco_cmd.yaw = 0.0;

        update_joint_status(q_cmd, q, qd);
        init_loco_policy_output_from_joint_cmd();
        construct_loco_single_obs();
        fill_loco_input_buffer();

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

    static void from_fsm_loco()
    {
        switch (ReqState)
        {
        case FSM_FAULT:
            FSMFaultEnter(FSM_LOCO);
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }
}
