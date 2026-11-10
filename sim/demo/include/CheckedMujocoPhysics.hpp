#pragma once

#include <array>
#include <cstddef>
#include <string>

class MujocoRobotBackend;

namespace jaka
{
// Safety boundary around the opaque SDK's physics step. The SDK ignores the
// AB2PitchRoll return code; every application physics substep must pass here.
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
    std::array<int, 4> qpos_addresses_{};
    std::array<int, 4> dof_addresses_{};
    bool stopped_ = false;
    std::string error_;
    std::size_t successful_steps_ = 0;
};
} // namespace jaka
