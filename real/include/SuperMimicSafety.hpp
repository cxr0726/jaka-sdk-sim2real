#pragma once

namespace RLController
{
    // Common SuperMimic feedback/IMU/inference/trajectory fault reporting.
    // Ankle kinematics use the original ParKine ABI directly in the FSM.
    void latch_super_fault(const char *where, const char *reason);
} // namespace RLController
