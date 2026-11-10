#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace jaka
{
// Physics owns the source mjData. The UI owns a separate model and displayed
// mjData, and never reads the backend. Rendering cannot hold the physics lock:
// a busy mailbox drops a display frame instead of delaying a control tick.
class ViewerSnapshot
{
    struct ModelDelete { void operator()(mjModel *p) const { if (p) mj_deleteModel(p); } };
    struct DataDelete { void operator()(mjData *p) const { if (p) mj_deleteData(p); } };

public:
    ViewerSnapshot(const mjModel *model, const mjData *data)
        : model_(model ? mj_copyModel(nullptr, model) : nullptr),
          pending_(model_ ? mj_makeData(model_.get()) : nullptr),
          displayed_(model_ ? mj_makeData(model_.get()) : nullptr)
    {
        if (!model_ || !pending_ || !displayed_ || !data)
            throw std::runtime_error("cannot allocate independent viewer snapshot");
        mj_copyData(pending_.get(), model_.get(), data);
        mj_copyData(displayed_.get(), model_.get(), data);
    }

    // Only the physics thread calls publish, between complete physics steps.
    bool publish(const mjData *source)
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return false;
        mj_copyData(pending_.get(), model_.get(), source);
        ++pending_sequence_;
        return true;
    }

    // Only the main/UI thread calls refresh and accesses model()/data().
    bool refresh()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (displayed_sequence_ == pending_sequence_) return false;
        mj_copyData(displayed_.get(), model_.get(), pending_.get());
        displayed_sequence_ = pending_sequence_;
        return true;
    }

    mjModel *model() const { return model_.get(); }
    mjData *data() const { return displayed_.get(); }

private:
    std::unique_ptr<mjModel, ModelDelete> model_;
    std::unique_ptr<mjData, DataDelete> pending_;
    std::unique_ptr<mjData, DataDelete> displayed_;
    std::mutex mutex_;
    std::uint64_t pending_sequence_{0};
    std::uint64_t displayed_sequence_{0};
};
} // namespace jaka
