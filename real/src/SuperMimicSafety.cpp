#include "SuperMimicSafety.hpp"
#include "RLController.hpp"

#include <cstdio>

namespace RLController
{
    void latch_super_fault(const char *where, const char *reason)
    {
        std::fprintf(stderr, "[SuperMimic fault] %s: %s; disabling PD and latching ERR_JOINT\n",
                     where ? where : "unknown", reason ? reason : "invalid input");
        mErrCode = ERR_JOINT;
        if (mRobot)
        {
            mRobot->enable_pd(false);
            mRobot->do_enable(false);
        }
    }
} // namespace RLController
