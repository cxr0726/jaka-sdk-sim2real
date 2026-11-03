#pragma once

// This header is used only by the standalone SuperMimicSafety test target.
// Put this directory before real/include on that target's include path. It
// intentionally has no ROS/RobotIntf implementation or hardware connection.
#include <memory>

namespace RLController
{
    enum ERRCODE { ERR_NONE, ERR_INIT, ERR_STATE_TRANSITION, ERR_JOINT, ERR_ABORT };

    struct StubRobot
    {
        int pd_off = 0;
        int enable_off = 0;
        void enable_pd(bool enabled) { if (!enabled) ++pd_off; }
        void do_enable(bool enabled) { if (!enabled) ++enable_off; }
    };

    extern std::unique_ptr<StubRobot> mRobot;
    extern int mNumAction;
    extern ERRCODE mErrCode;
} // namespace RLController
