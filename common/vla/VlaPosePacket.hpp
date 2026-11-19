#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace RLController
{
    struct VlaPoseFrame
    {
        std::int64_t index;
        std::array<float, 27> joints;     // Grouped JAKA/MuJoCo wire order, radians.
        std::array<float, 3> position;   // Reference base_link in world coordinates, metres.
        std::array<float, 4> quaternion; // Normalized wxyz, base_link to world.
    };

    struct VlaPosePacket
    {
        std::vector<VlaPoseFrame> frames;
        std::string stream_id;
    };

    inline constexpr std::size_t kVlaMaxPacketBytes = 64 * 1024;
    inline constexpr std::size_t kVlaMaxPacketFrames = 16;

    // On failure out retains its previous value. Joint reordering and time
    // assignment are the controller's responsibility.
    bool decode_vla_pose(const void *data, std::size_t size, VlaPosePacket &out);
} // namespace RLController
