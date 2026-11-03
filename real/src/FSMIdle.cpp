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
    static bool super_combo_armed;
    static legged_waist_controller::RobotIntf::ControlSource ctrl_source;

    static void from_fsm_idle();

    static void fsm_idle_judge()
    {
        // RobotIntf owns control-source switching and the watchdog-protected Joy cache.
        ctrl_source=mRobot->get_control_source();
        bool joy_read = false;
        if(ctrl_source==legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            joy_read = mRobot->get_joy_robot_state(joydata);
        }

        // LB+B also returns the older MIMIC mode to IDLE. Require a release
        // after IDLE entry and then a new press, so that held return gesture
        // cannot immediately start SuperMimic. Consume one attempt per press.
        const bool joy_online =
            ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY &&
            joy_read && joydata.joy_online;
        const bool super_combo = joy_online && joydata.joy.buttons_LB && joydata.joy.buttons_B;
        const bool super_pressed = super_combo && super_combo_armed;
        if (!joy_online || super_pressed)
            super_combo_armed = false;
        else if (!super_combo)
            super_combo_armed = true;

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
                else if (mJakaMiniSuperZmqEnabled && super_pressed && !joydata.joy.buttons_A)
                {
                    // SuperMimic entry checks feedback/IMU again before use.
                    // Never enable joints implicitly as part of mode selection.
                    if (mDataReadyFlag && mEnabledJointCount == mNumAction && mFaultJointCount == 0)
                        ReqState = FSM_JAKAMINIMIMIC_SUPERZMQ;
                    else
                        printf("[SuperMimic] entry ignored: enable all joints and clear faults, then press LB+B again\n");
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
        update_joint_status(q_cmd, q, qd);
        if (mFaultJointCount > 0)
        {
            mErrCode = ERR_JOINT;
        }

        if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
        {
            if (!enable_robot && joydata.joy.buttons_BACK)
            {
                // BACK requests the controller's all-joint enable operation.
                mRobot->do_enable(true);
                enable_robot = true;
                printf("------------enable robot-----------\n");
            }

            if (enable_robot && joydata.joy.buttons_A)
            {
                // A requests all-joint disable and cancels the local target move.
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
            printf("-------------------init joint state bind-----------------\n");
        }

        if (mEnabledJointCount == mNumAction)
        {
            int i;
            if (ctrl_source == legged_waist_controller::RobotIntf::ControlSource::JOY)
            {
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

            if (go_target)
            {
                for (i = 0; i < mNumAction; i++)
                {
                    mNlf2[i].step(&mNlf2[i], target[i], 0.0);
                    mPt.positions[i] = mNlf2[i].x0;
                }
                mPt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(mTimeStep));
                // Publish one point with names and positions in the configured joint order.
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
        super_combo_armed = false;
        // Keep RobotIntf/PiState synchronized with the local FSM.
        mRobot->set_cur_control_stat(FSM_IDLE);
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
