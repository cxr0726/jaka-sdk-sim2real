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
        if (StateJudge && StateRun)
        {
            StateJudge();
            StateRun();
        }
    }
}