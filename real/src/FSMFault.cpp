#include "RLController.hpp"
#include <stdio.h>

namespace RLController
{

    extern fsm_func_ptr StateJudge;
    extern fsm_func_ptr StateRun;
    extern FSMSTATE ReqState;

    static legged_waist_controller::RobotIntf::JoyRobotState joydata;
    static double q_cmd[50], q[50], qd[50];
    static legged_waist_controller::RobotIntf::ControlSource ctrl_source;

    static void from_fsm_fault();

    static void fsm_fault_judge()
    {
        // Read the current input owner and, for JOY, its watchdog-protected snapshot.
        ctrl_source=mRobot->get_control_source();
        if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            mRobot->get_joy_robot_state(joydata);
        }

        update_joint_status(q_cmd, q, qd);
        if (mFaultJointCount == 0)
        {
            if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
            {
                if (joydata.joy.buttons_START)
                {
                    mErrCode = ERR_NONE;
                    ReqState = FSM_INIT;
                    printf("----------- FSM reset -----------\n");
                }
            }
        }
        else
        {
            if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
            {
                if (joydata.joy.buttons_START)
                {
                    // Pulse the controller's all-joint fault-reset operation.
                    mRobot->clear_error();
                }
            }
        }

        if (ReqState != FSM_FAULT)
        {
            from_fsm_fault();
        }
    }

    static void fsm_fault_run()
    {
        if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            if (mLogData && joydata.joy.buttons_X)
            {
                mLogData = 0;
                printf("----------- data log stopped -----------\n");
            }
        }
    }

    void FSMFaultEnter(FSMSTATE from)
    {
        mFSMState = ReqState = FSM_FAULT;
        StateJudge = &fsm_fault_judge;
        StateRun = &fsm_fault_run;
        // Fault entry immediately requests servo-off before any recovery attempt.
        mRobot->do_enable(false);
        // Report FAULT through RobotIntf's current-control-state output.
        mRobot->set_cur_control_stat(FSM_FAULT);
        joydata = legged_waist_controller::RobotIntf::JoyRobotState{};

        if (mDebugFlag)
            printf("--------from state %d to state %d------------\n", from, FSM_FAULT);

        switch (from)
        {
        case FSM_IDLE:
            break;
        case FSM_LOCO:
            break;
        case FSM_MIMIC:
            break;
        case FSM_JAKAMINIMIMIC_ZMQ:
            break;
        case FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST:
            break;
        case FSM_JAKAMINIMIMIC_SUPERZMQ:
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }

    static void from_fsm_fault()
    {
        switch (ReqState)
        {
        case FSM_IDLE:
            FSMIdleEnter(FSM_FAULT);
            break;
        case FSM_INIT:
            FSMInitEnter(FSM_FAULT);
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }
}
