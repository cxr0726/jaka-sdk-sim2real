#include "CheckedMujocoPhysics.hpp"
#include "MujocoRobotBackend.hpp"

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

        backend_.step_physics();
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
