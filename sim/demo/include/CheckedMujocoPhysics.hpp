#pragma once

#include <cstddef>
#include <string>

class MujocoRobotBackend;

namespace jaka
{
// Generic finite-state/exception boundary around the opaque SDK physics step.
// Ankle kinematics are handled by the original ParKine ABI inside the SDK;
// this wrapper does not impose a second assembly-root or convergence check.
// The application must be the sole writer/stepper of this backend's mjData.
class CheckedMujocoPhysics
{
public:
    explicit CheckedMujocoPhysics(MujocoRobotBackend &backend);
    bool step() noexcept;
    void shutdown() noexcept;
    const std::string &error() const noexcept { return error_; }
    std::size_t successful_steps() const noexcept { return successful_steps_; }

private:
    bool fail(const char *message) noexcept;
    MujocoRobotBackend &backend_;
    bool stopped_ = false;
    std::string error_;
    std::size_t successful_steps_ = 0;
};
} // namespace jaka
