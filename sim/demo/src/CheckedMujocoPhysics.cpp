#include "CheckedMujocoPhysics.hpp"
#include "MujocoRobotBackend.hpp"
#include "ParKine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>

namespace
{
bool all_finite(const mjtNum *values, int count) noexcept
{
    for (int i = 0; i < count; ++i)
        if (!std::isfinite(values[i])) return false;
    return true;
}
}

namespace jaka
{
CheckedMujocoPhysics::CheckedMujocoPhysics(MujocoRobotBackend &backend) : backend_(backend)
{
    error_.reserve(512);
    const mjModel *model = backend_.model();
    if (!model || !backend_.data())
        throw std::runtime_error("CheckedMujocoPhysics requires an initialized backend");
    const char *names[] = {"Left_drive1_joint", "Left_drive2_joint",
                           "Right_drive1_joint", "Right_drive2_joint"};
    for (int i = 0; i < 4; ++i)
    {
        const int joint = mj_name2id(model, mjOBJ_JOINT, names[i]);
        if (joint < 0 || model->jnt_type[joint] != mjJNT_HINGE)
            throw std::runtime_error(std::string("Missing parallel ankle drive: ") + names[i]);
        qpos_addresses_[i] = model->jnt_qposadr[joint];
        dof_addresses_[i] = model->jnt_dofadr[joint];
    }
}

void CheckedMujocoPhysics::shutdown() noexcept
{
    stopped_ = true;
    // Clear commands immediately, including when disabling the backend throws.
    if (backend_.model() && backend_.data())
        std::fill_n(backend_.data()->ctrl, backend_.model()->nu, mjtNum(0));
    try { backend_.enable_pd(false); } catch (...) {}
    try { backend_.do_enable(false); } catch (...) {}
    if (backend_.model() && backend_.data())
        std::fill_n(backend_.data()->ctrl, backend_.model()->nu, mjtNum(0));
}

bool CheckedMujocoPhysics::fail(const char *message) noexcept
{
    // reserve(512) in the constructor keeps ordinary fault recording allocation
    // free. Messages are bounded, and shutdown still happens on allocation error.
    const char *detail = message ? message : "unknown physics safety failure";
    try { error_.assign(detail, std::min<std::size_t>(std::char_traits<char>::length(detail), 400)); }
    catch (...) {}
    shutdown();
    return false;
}

bool CheckedMujocoPhysics::step() noexcept
{
    if (stopped_) return false;
    try
    {
        const mjModel *model = backend_.model();
        mjData *data = backend_.data();
        if (!model || !data) return fail("MuJoCo model/data became unavailable");
        if (!all_finite(data->qpos, model->nq) || !all_finite(data->qvel, model->nv))
            return fail("non-finite physical joint state before SDK PD");
        if (!all_finite(data->ctrl, model->nu))
            return fail("non-finite residual control before SDK PD");

        // Use precisely the stateless C ABI called through the SDK's PLT. A
        // stateful warm seed could choose another assembly root; it is therefore
        // deliberately not substituted for this preflight. No physics or state
        // mutation occurs between these checks and the synchronous SDK call.
        for (int side = 0; side < 2; ++side)
        {
            const int i = 2 * side;
            const double sign = side == 0 ? 1.0 : -1.0;
            double pitch, roll, dpitch, droll, jac[2][2];
            const int code = ::AB2PitchRoll(
                sign * data->qpos[qpos_addresses_[i]],
                sign * data->qpos[qpos_addresses_[i + 1]],
                sign * data->qvel[dof_addresses_[i]],
                sign * data->qvel[dof_addresses_[i + 1]],
                &pitch, &roll, &dpitch, &droll, jac);
            if (code < 0 || !std::isfinite(pitch) || !std::isfinite(roll) ||
                !std::isfinite(dpitch) || !std::isfinite(droll) ||
                !std::isfinite(jac[0][0]) || !std::isfinite(jac[0][1]) ||
                !std::isfinite(jac[1][0]) || !std::isfinite(jac[1][1]))
            {
                char message[160];
                std::snprintf(message, sizeof(message), "%s ankle solve failed before SDK PD (code=%d)",
                              side == 0 ? "left" : "right", code);
                return fail(message);
            }
        }

        backend_.step_physics();
        // This is an additional diagnostic, not the substitute for preflight.
        if (!all_finite(data->qpos, model->nq) || !all_finite(data->qvel, model->nv) ||
            !all_finite(data->ctrl, model->nu))
            return fail("non-finite result after SDK physics step");
        ++successful_steps_;
        return true;
    }
    catch (const std::exception &exception)
    {
        char message[400];
        std::snprintf(message, sizeof(message), "SDK physics exception: %.350s", exception.what());
        return fail(message);
    }
    catch (...) { return fail("unknown SDK physics exception"); }
}
} // namespace jaka
