#include "ViewerSnapshot.hpp"

#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
}

int main(int argc, char **argv)
{
    try
    {
        require(argc == 2, "expected XML path");
        char error[1024]{};
        mjModel *model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
        require(model != nullptr, error);
        mjData *source = mj_makeData(model);
        require(source != nullptr, "cannot allocate test state");
        {
            jaka::ViewerSnapshot snapshot(model, source);
            require(snapshot.model() != model && snapshot.data() != source,
                    "viewer shares live physics memory");
            source->time = 1.;
            source->qpos[0] = 2.;
            require(snapshot.publish(source), "uncontended publish failed");
            require(snapshot.data()->time == 0., "publish mutated displayed state during rendering");
            require(snapshot.refresh(), "new snapshot missing");
            require(snapshot.data()->time == 1. && snapshot.data()->qpos[0] == 2., "snapshot data mismatch");
            require(!snapshot.refresh(), "unchanged snapshot reported new frame");
            source->qpos[0] = 3.;
            require(snapshot.data()->qpos[0] == 2., "UI reads source state without a snapshot");

            // Deliberately render/read an older frame while physics publishes.
            // Each mailbox copy must be coherent, never partially updated.
            std::atomic<bool> done{false};
            std::thread producer([&]()
            {
                for (int i = 2; i <= 10000; ++i)
                {
                    source->time = i;
                    source->qpos[0] = 2. * i;
                    source->qvel[0] = -3. * i;
                    snapshot.publish(source);
                }
                done.store(true, std::memory_order_release);
            });
            bool coherent = true;
            while (!done.load(std::memory_order_acquire))
            {
                if (snapshot.refresh() && snapshot.data()->time >= 2.)
                    coherent = coherent && snapshot.data()->qpos[0] == 2. * snapshot.data()->time &&
                               snapshot.data()->qvel[0] == -3. * snapshot.data()->time;
            }
            producer.join();
            require(coherent, "torn viewer snapshot");
            require(snapshot.publish(source), "publish remained busy after UI stopped");
            snapshot.refresh();
            require(snapshot.data()->time == 10000. && snapshot.data()->qpos[0] == 20000.,
                    "final snapshot missing");
        }
        mj_deleteData(source);
        mj_deleteModel(model);
        std::cout << "independent model/state, display isolation and concurrent snapshot coherence passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
