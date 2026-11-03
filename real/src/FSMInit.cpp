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
        // RobotIntf readiness requires both joint feedback and at least one cached IMU sample.
        if (mDataReadyFlag && mRobot->has_imu())
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
        legged_waist_controller::RobotIntf::JointStatus joint_state;
        int i, op_count=0;
        for (i = 0; i < mNumAction; i++)
        {
            // Poll cached controller status until every configured EtherCAT slave is OP.
            mRobot->get_joint_status(mJointNames[i], joint_state);
            if(joint_state.ethercat_operational)
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
        mFSMState = ReqState = FSM_INIT;

        StateJudge = &fsm_init_judge;
        StateRun = &fsm_init_run;
        mRLThreadRunFlag = true;
        mDataReadyFlag = false;
        // Expose this FSM position through RobotIntf's current-control-state output.
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

            // joint limits settings
            for (i = 0; i < mNumAction; i++)
            {
                // Send each configured joint's software limit to the controller.
                mRobot->set_joint_angle_limit(mJointNames[i], -6.3, 6.3);
            }

            // pd and gain settings, should move to the state run due to rt requirements
            for(i=0;i<mNumAction;i++)
            {
                // Operation mode is controller-wide; MIT gains are then set per joint.
                mRobot->set_joint_operation_mode(legged_waist_controller::RobotIntf::JOINT_OPERATION_MODE_MIT);
                mRobot->set_joint_mit_gains(mJointNames[i], mKp[i], mKd[i]);
            }

            if (mAngleMapFlag)
            {
                // Disable the controller's separate PD path when action angle mapping is active.
                mRobot->enable_pd(false);
            }
            else
            {
                // Four coupled ankle axes use the controller PD path instead of MIT gains.
                i = 4;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 5);
                i = 5;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 5);
                i = 16;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 5);
                i = 17;
                mRobot->set_joint_mit_gains(mJointNames[i], 0, 5);
                // Enable PD globally before loading the four per-joint PD gain pairs.
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

            // nlf2 planner settings
            bind_nlf2_state();

            FSMIdleEnter(FSM_INIT);
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }
}
