#include "RLController.hpp"
#include <stdio.h>

namespace RLController
{

    extern fsm_func_ptr StateJudge;
    extern fsm_func_ptr StateRun;
    extern FSMSTATE ReqState;

    static void from_fsm_init();

    static void fsm_init_judge()
    {
        // legged_waist_controller::
        // auto fsm_state = mRobot->get_latest_mot_command();
        // if (fsm_state == legged_waist_controller::RobotMotType::MOT_TYPE_NONE)
        // {
        //     printf("nothing_happened\n");
        // }
        // legged_waist_controller::RobotIntf::LocoMoveCmd cmd{};
        if (mDataReadyFlag && mRobot->has_imu()) // && mRobot->get_latest_loco_move(cmd))
        {
            ReqState = FSM_IDLE;
        }

        if (ReqState != FSM_INIT)
        {
            from_fsm_init();
        }
    }

    static void fsm_init_run()
    {
        // wait for op state
        // printf("op_state: ");
        int i, op_count=0;
        for (i = 0; i < mNumAction; i++)
        {
            legged_waist_controller::RobotIntf::JointStatus joint_state{};
            if(mRobot->get_joint_status(mJointNames[i], joint_state) >= 0 && joint_state.ethercat_operational)
            {
                op_count++;
            }
        }
        if(!mDataReadyFlag && op_count==mNumAction)
        {
            mDataReadyFlag=true;
            printf("------------all joints are in op mode----------------\n");
        }

    }

    void FSMInitEnter(FSMSTATE from)
    {
        reset_parallel_ankle_observers();
        mFSMState = ReqState = FSM_INIT;

        StateJudge = &fsm_init_judge;
        StateRun = &fsm_init_run;
        mRLThreadRunFlag = true;
        mDataReadyFlag = false;
        mRobot->set_cur_control_stat(FSM_INIT);

        if (mDebugFlag)
            printf("--------from state %d to state %d------------\n", from, FSM_INIT);

        switch (from)
        {
        case FSM_INIT:
            break;
        case FSM_FAULT:
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }

    static void from_fsm_init()
    {
        int i = 0;
        switch (ReqState)
        {
        case FSM_IDLE: // allow time out
            /*
                to do:
                1. directions settings
                2. joint limits settings
                3. pd and gain settings
                4. nlf2 planner settings

            */
            // 1. direction settings
            // i = 0;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 2;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 3;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 4;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 5;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 6;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 7;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 8;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 9;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 10;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 11;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 12;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 13;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 13;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 14;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 15;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 16;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 17;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 18;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 19;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 20;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 21;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 22;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 23;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 24;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 25;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);
            // i = 26;
            // mRobot->set_joint_command_direction(mJointNames[i], 1);
            // mRobot->set_joint_feedback_direction(mJointNames[i], 1);

            // 2. joint limits settings
            for (i = 0; i < mNumAction; i++)
            {
                mRobot->set_joint_angle_limit(mJointNames[i], -6.3, 6.3);
            }

            // 3. pd and gain settings, should move to the state run due to rt requirements
            for(i=0;i<mNumAction;i++)
            {
                mRobot->set_joint_operation_mode(legged_waist_controller::RobotIntf::JOINT_OPERATION_MODE_MIT);
                mRobot->set_joint_mit_gains(mJointNames[i], mKp[i], mKd[i]);
            }
            // mAngleMapFlag=true;
            if (mAngleMapFlag)
            {
                mRobot->enable_pd(false);
            }
            else
            {
                i = 4;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 0);
                i = 5;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 0);
                i = 16;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 0);
                i = 17;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 0);
                mRobot->enable_pd(true);
                i = 4;
                mRobot->set_joint_pd_para(mJointNames[i], mKp_PR[0], mKd_PR[0]);
                i = 5;
                mRobot->set_joint_pd_para(mJointNames[i], mKp_PR[1], mKd_PR[1]);
                i = 16;
                mRobot->set_joint_pd_para(mJointNames[i], mKp_PR[2], mKd_PR[2]);
                i = 17;
                mRobot->set_joint_pd_para(mJointNames[i], mKp_PR[3], mKd_PR[3]);
            }

            // 4. nlf2 planner settings
            bind_nlf2_state();
            if (mErrCode != ERR_NONE) return;

            FSMIdleEnter(FSM_INIT);
            break;
        // case RL_FAULT:
        //     rl_fault_init(RL_INIT);
        //     break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }
}
