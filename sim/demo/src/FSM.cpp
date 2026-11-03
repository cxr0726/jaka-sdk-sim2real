#include "RLController.hpp"
#include <stdlib.h>

namespace RLController
{
    /**** scope within fsm, not exposed to public */
    fsm_func_ptr StateJudge = NULL;
    fsm_func_ptr StateRun = NULL;
    FSMSTATE ReqState = FSM_INIT;

    void FSMUpdate()
    {
        if (mErrCode == ERR_NONE && !mRLThreadRunFlag)
        {
            mErrCode = ERR_ABORT;
        }

        if (StateJudge && StateRun)
        {
            StateJudge();
            if (mErrCode != ERR_NONE && mFSMState != FSM_FAULT)
            {
                FSMFaultEnter(mFSMState);
                return;
            }
            StateRun();
        }
    }

}
