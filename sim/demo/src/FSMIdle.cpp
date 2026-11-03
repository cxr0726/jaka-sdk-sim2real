#include "RLController.hpp"
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace RLController
{

    extern fsm_func_ptr StateJudge;
    extern fsm_func_ptr StateRun;
    extern FSMSTATE ReqState;
    static bool bind_data;
    static bool go_target;
    static double target[50], q_cmd[50], q[50], qd[50];
    static legged_waist_controller::RobotIntf::JoyRobotState joydata;
    static bool enable_robot;
    static legged_waist_controller::RobotIntf::ControlSource ctrl_source;
    static MOT_TYPE_CODE mot_type_code;
    static legged_waist_controller::MotCommand mot_command;

    static void from_fsm_idle();

    static void fsm_idle_judge()
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

        if (mErrCode != ERR_NONE)
        {
            ReqState = FSM_FAULT;
        }
        else
        {
            if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
            {
                if (joydata.joy.buttons_LB && joydata.joy.buttons_Y)
                {
                    ReqState = FSM_LOCO;
                }
                else if (joydata.joy.buttons_LB && joydata.joy.buttons_X)
                {
                    ReqState = FSM_MIMIC;
                }
                else if (mJakaMiniZmqEnabled && joydata.joy.buttons_LB && joydata.joy.buttons_START)
                {
                    ReqState = FSM_JAKAMINIMIMIC_ZMQ;
                }
                else if (mJakaMiniZmqFixwristEnabled && joydata.joy.buttons_LB && joydata.joy.buttons_BACK)
                {
                    ReqState = FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST;
                }
            }
            else if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::APP)
            {
                switch(mot_type_code)
                {
                    case MOT_TYPE_SWITCH_TO_LOCO:ReqState=FSM_LOCO;break;
                    case MOT_TYPE_SWITCH_TO_MIMIC:ReqState=FSM_MIMIC;break;
                    default:break;
                }
            }
        }

        if (ReqState != FSM_IDLE)
        {
            from_fsm_idle();
        }
    }

    static void fsm_idle_run()
    {
        // to do: wait for
        // if (mDebugFlag)
        //     printf("----------I'm in FSM_IDLE------------\n");

        update_joint_status(q_cmd, q, qd);
        if (mFaultJointCount > 0)
        {
            latch_ankle_fault("idle feedback", "joint feedback fault");
            return;
        }

        if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            if (!enable_robot && joydata.joy.buttons_BACK)
            {
                mRobot->do_enable(true);
                enable_robot = true;
                printf("------------enable robot-----------\n");
            }

            if (enable_robot && joydata.joy.buttons_A)
            {
                mRobot->do_enable(false);
                enable_robot = false;
                bind_data = false;
                go_target = false;
                printf("---------------- disable robot ---------------\n");
            }
        }
        else if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::APP)
        {
            if(!enable_robot && mot_type_code==MOT_TYPE_ENABLE)
            {
                mRobot->do_enable(true);
                enable_robot = true;
                printf("------------enable robot-----------\n");
            }

            if(enable_robot && mot_type_code==MOT_TYPE_DISABLE)
            {
                mRobot->do_enable(false);
                enable_robot = false;
                bind_data = false;
                go_target = false;
                printf("---------------- disable robot ---------------\n");
            }
        }

        if (mEnabledJointCount == mNumAction && 
            enable_robot && !bind_data && mErrCode == ERR_NONE) // at enbaled
        {
            int i;
            for (i = 0; i < mNumAction; i++)
            {
                mNlf2[i].x0 = q_cmd[i];
                mNlf2[i].v0 = 0;
                printf("%lf ", q_cmd[i]);
            }
            printf("\n");
            bind_data = true;
            printf("-------------enable data bind-----------------\n");
        }

        if (mEnabledJointCount == mNumAction)
        {
            int i;
            if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
            {
                if (joydata.joy.axes_DX > 0.5f)
                {
                    for (i = 0; i < mNumAction; i++)
                    {
                        target[i] = mPosTarget[i];
                    }
                    go_target = true;
                }
                else if (joydata.joy.axes_DX < -0.5f)
                {
                    for (i = 0; i < mNumAction; i++)
                    {
                        target[i] = mNegTarget[i];
                    }
                    go_target = true;
                }
                if (joydata.joy.axes_DY > 0.5f)
                {
                    for (i = 0; i < mNumAction; i++)
                    {
                        target[i] = mZeroTarget[i];
                    }
                    go_target = true;
                }
                else if (joydata.joy.axes_DY < -0.5f)
                {
                    for (i = 0; i < mNumAction; i++)
                    {
                        target[i] = mInitTarget[i];
                    }
                    go_target = true;
                }
            }
            else if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::APP)
            {
                if(mot_type_code==MOT_TYPE_GO_HOME)
                {
                    auto home_pos=mRobot->get_home_positions();
                    if(home_pos.size()==(unsigned int)mNumAction)
                    {
                        for (i = 0; i < mNumAction; i++)
                        {
                            target[i] = home_pos[i];
                        }
                    }
                    else
                    {
                        for (i = 0; i < mNumAction; i++)
                        {
                            target[i] = mZeroTarget[i];
                        }
                    }
                    go_target = true;
                }
                else if(mot_type_code==MOT_TYPE_GO_LOCO_INITPOS)
                {
                    for (i = 0; i < mNumAction; i++)
                    {
                        target[i] = mInitTarget[i];
                    }
                    go_target = true;
                }
            }

            if (go_target)
            {
                for (i = 0; i < mNumAction; i++)
                {
                    mNlf2[i].step(&mNlf2[i], target[i], 0.0);
                    mPt.positions[i] = mNlf2[i].x0;
                }
                mPt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(mTimeStep));
                mRobot->set_joint_trajectory(mJointNames, mPt);
            }
        }
    }

    void FSMIdleEnter(FSMSTATE from)
    {
        mFSMState = ReqState = FSM_IDLE;
        StateJudge = &fsm_idle_judge;
        StateRun = &fsm_idle_run;
        bind_data = false;
        go_target = false;
        enable_robot = false;
        mRobot->set_cur_control_stat(FSM_IDLE);
        mot_type_code=MOT_TYPE_NONE;
        joydata = legged_waist_controller::RobotIntf::JoyRobotState{};
        int i;
        for (i = 0; i < mNumAction; i++)
        {
            target[i] = mInitTarget[i];
        }

        if (mDebugFlag)
            printf("--------[mDebugFlag] from state %d to state %d------------\n", from, FSM_IDLE);

        switch (from)
        {
        case FSM_INIT:
            break;
        case FSM_FAULT:
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

    static void from_fsm_idle()
    {
        switch (ReqState)
        {
        case FSM_LOCO:
            FSMLocoEnter(FSM_IDLE);
            break;
        case FSM_MIMIC:
            FSMMimicEnter(FSM_IDLE);
            break;
        case FSM_JAKAMINIMIMIC_ZMQ:
            FSMMimicJakaMiniZmqEnter(FSM_IDLE);
            break;
        case FSM_JAKAMINIMIMIC_ZMQ_FIXWRIST:
            FSMMimicJakaMiniZmqFixwristEnter(FSM_IDLE);
            break;
        case FSM_JAKAMINIMIMIC_SUPERZMQ:
            FSMMimicJakaMiniSuperZmqEnter(FSM_IDLE);
            break;
        case FSM_FAULT:
            FSMFaultEnter(FSM_IDLE);
            break;
        default:
            mErrCode = ERR_STATE_TRANSITION;
            break;
        }
    }
}
