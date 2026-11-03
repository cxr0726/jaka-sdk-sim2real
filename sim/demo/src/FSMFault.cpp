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
    static MOT_TYPE_CODE mot_type_code;
    static legged_waist_controller::MotCommand mot_command;

    static void from_fsm_fault();

    static void fsm_fault_judge()
    {
        ctrl_source=mRobot->get_control_source();
        if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            mRobot->get_joy_robot_state(joydata);
        }
        else if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::APP){
            mot_type_code=MOT_TYPE_NONE;

            if(mRobot->poll_mot_command(mot_command))
            {
                mot_type_code=(MOT_TYPE_CODE)mot_command.mot_type;
            }
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
            else if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::APP)
            {
                if(mot_type_code==MOT_TYPE_RESET)
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
                    mRobot->clear_error();
                }
            }
            else if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::APP)
            {
                mRobot->clear_error();
            }
        }

        if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            if (mLogData && joydata.joy.buttons_X)
            {
                mLogData = 0;
                printf("----------- data log stopped -----------\n");
            }
        }

        if (ReqState != FSM_FAULT)
        {
            from_fsm_fault();
        }
    }

    static void fsm_fault_run()
    {
        // to do: wait for
        // if (mDebugFlag)
        //     printf("----------I'm in FSM_FAULT------------\n");
    }

    void FSMFaultEnter(FSMSTATE from)
    {
        mFSMState = ReqState = FSM_FAULT;
        StateJudge = &fsm_fault_judge;
        StateRun = &fsm_fault_run;
        mRobot->do_enable(false);
        mRobot->set_cur_control_stat(FSM_FAULT);
        mot_type_code=MOT_TYPE_NONE;
        joydata = legged_waist_controller::RobotIntf::JoyRobotState{};

        if (mDebugFlag)
            printf("--------from state %d to state %d------------\n", from, FSM_FAULT);

        switch (from)
        {
        case FSM_INIT:
            break;
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
