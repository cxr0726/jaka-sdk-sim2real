#pragma once

namespace RLController
{
    // These helpers belong only to the new SuperMimic state. They do not
    // replace the legacy ParKine C ABI used by the existing real modes.
    // Arrays must contain all 27 joints in deployment order. Conversion is
    // transactional: a failed ankle leaves both arrays/targets unchanged.
    bool decode_super_parallel_ankles(double *q, double *qd, const char *where);
    bool encode_super_parallel_ankle_targets(double *q, const char *where);

    // Latch ERR_JOINT and immediately disable PD and robot enable. No failed
    // feedback/target may be replaced with zeros or a stale previous solution.
    void latch_super_fault(const char *where, const char *reason);
    void reset_super_parallel_ankles();
} // namespace RLController
