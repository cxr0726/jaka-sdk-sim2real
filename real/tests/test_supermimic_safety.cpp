// Common fault latch only. Ankle behavior is exercised through the actual FSM.
#include "RLController.hpp"
#include "SuperMimicSafety.hpp"
#include <iostream>
#include <stdexcept>

namespace RLController
{
    std::unique_ptr<StubRobot> mRobot;
    int mNumAction = 27;
    ERRCODE mErrCode = ERR_NONE;
}

int main()
{
    using namespace RLController;
    try
    {
        latch_super_fault(nullptr, nullptr);
        if (mErrCode != ERR_JOINT) throw std::runtime_error("null-robot fault not latched");
        mRobot = std::make_unique<StubRobot>();
        mErrCode = ERR_NONE;
        latch_super_fault("offline test", "injected failure");
        if (mErrCode != ERR_JOINT || mRobot->pd_off != 1 || mRobot->enable_off != 1)
            throw std::runtime_error("fault omitted PD/servo-off or error latch");
        std::cout << "Common fault latch passed with null/fake robot; no hardware\n";
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
