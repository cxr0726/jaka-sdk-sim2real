#include "VlaPosePacket.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include <nlohmann/json.hpp>

namespace RLController
{
    namespace
    {
        using Json = nlohmann::json;
        constexpr std::size_t kTopicBytes = 4;
        constexpr std::size_t kHeaderBytes = 1280;

        // Read the specified little-endian wire representation on any host.
        template <typename Uint>
        Uint read_le(const std::uint8_t *data)
        {
            Uint value = 0;
            for (std::size_t i = 0; i < sizeof(Uint); ++i)
                value |= static_cast<Uint>(data[i]) << (8 * i);
            return value;
        }

        float read_float(const std::uint8_t *data)
        {
            static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
            const std::uint32_t bits = read_le<std::uint32_t>(data);
            float value;
            std::memcpy(&value, &bits, sizeof(value));
            return value;
        }

        template <std::size_t N>
        bool read_vector(const std::uint8_t *data, std::array<float, N> &result)
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                result[i] = read_float(data + i * sizeof(float));
                if (!std::isfinite(result[i]))
                    return false;
            }
            return true;
        }

        struct Field
        {
            const char *name;
            const char *dtype;
            std::size_t width;
            std::size_t element_bytes;
            const std::uint8_t *data = nullptr;
        };

        bool decode(const std::uint8_t *bytes, std::size_t size, VlaPosePacket &out)
        {
            if (bytes == nullptr || size <= kTopicBytes + kHeaderBytes ||
                size > kVlaMaxPacketBytes || std::memcmp(bytes, "pose", kTopicBytes) != 0)
                return false;
            const auto *header_begin = bytes + kTopicBytes;
            const auto *payload = header_begin + kHeaderBytes;
            const auto *json_end = std::find(header_begin, payload, std::uint8_t{0});
            if (!std::all_of(json_end, payload, [](std::uint8_t b) { return b == 0; }))
                return false;
            const auto header = Json::parse(header_begin, json_end, nullptr, false);
            if (!header.is_object() || header.at("v") != 1 || !header.at("v").is_number_unsigned() ||
                header.at("endian") != "le" || header.at("anchor_body") != "base_link" ||
                header.at("joint_order") != "jaka" ||
                !header.at("stream_id").is_string() || !header.at("fields").is_array())
                return false;
            // In OpenHLM this counts messages, not the sliding window's rows.
            if (header.contains("count") &&
                (!header.at("count").is_number_unsigned() || header.at("count") != 1))
                return false;
            const auto &stream = header.at("stream_id").get_ref<const std::string &>();
            if (stream.empty() || stream.size() > 128 ||
                !std::all_of(stream.begin(), stream.end(), [](unsigned char c) { return c >= 33 && c <= 126; }))
                return false;

            std::array<Field, 5> schema{{
                {"joint_pos", "f32", 27, 4}, {"body_pos_w", "f32", 3, 4},
                {"body_quat_w", "f32", 4, 4}, {"frame_index", "i64", 1, 8},
                {"joint_vel", "f32", 27, 4},
            }};
            const auto &fields = header.at("fields");
            if (fields.size() < 4 || fields.size() > schema.size())
                return false;
            std::size_t rows = 0;
            std::size_t remaining = size - kTopicBytes - kHeaderBytes;
            for (const auto &field : fields)
            {
                const auto &name = field.at("name");
                const auto it = std::find_if(schema.begin(), schema.end(),
                                             [&name](const Field &f) { return name == f.name; });
                if (it == schema.end() || it->data != nullptr || field.at("dtype") != it->dtype)
                    return false;
                const auto &shape = field.at("shape");
                if (!shape.is_array() || shape.size() != (it->width == 1 ? 1U : 2U) ||
                    !shape[0].is_number_unsigned())
                    return false;
                const auto n = shape[0].get<std::uint64_t>();
                if (n == 0 || n > kVlaMaxPacketFrames || (rows != 0 && rows != n))
                    return false;
                if (it->width != 1 && (!shape[1].is_number_unsigned() || shape[1] != it->width))
                    return false;
                rows = static_cast<std::size_t>(n);
                // All three factors are bounded: rows <= 16, width <= 27, element_bytes <= 8.
                const std::size_t length = rows * it->width * it->element_bytes;
                if (length > remaining)
                    return false;
                it->data = payload;
                payload += length;
                remaining -= length;
            }
            if (remaining != 0 || !schema[0].data || !schema[1].data || !schema[2].data || !schema[3].data)
                return false;

            VlaPosePacket next;
            next.stream_id = stream;
            next.frames.resize(rows);
            std::uint64_t previous = 0;
            for (std::size_t row = 0; row < rows; ++row)
            {
                VlaPoseFrame &frame = next.frames[row];
                const auto index = read_le<std::uint64_t>(schema[3].data + row * 8);
                if (index > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
                    (row > 0 && index != previous + 1))
                    return false;
                frame.index = static_cast<std::int64_t>(index);
                previous = index;
                if (!read_vector(schema[0].data + row * 27 * 4, frame.joints) ||
                    !read_vector(schema[1].data + row * 3 * 4, frame.position) ||
                    !read_vector(schema[2].data + row * 4 * 4, frame.quaternion))
                    return false;
                double norm_squared = 0;
                for (float value : frame.quaternion)
                    norm_squared += static_cast<double>(value) * value;
                // The sender generates unit quaternions. Tolerate small rounding
                // and reject a corrupt orientation instead of replacing it with identity.
                const double norm = std::sqrt(norm_squared);
                if (std::abs(norm - 1.0) > 0.01)
                    return false;
                for (float &value : frame.quaternion)
                    value = static_cast<float>(value / norm);

                std::array<float, 27> velocity;
                if (schema[4].data && !read_vector(schema[4].data + row * 27 * 4, velocity))
                    return false;
            }
            out = std::move(next);
            return true;
        }
    } // namespace

    bool decode_vla_pose(const void *data, std::size_t size, VlaPosePacket &out)
    {
        try
        {
            return decode(static_cast<const std::uint8_t *>(data), size, out);
        }
        catch (const Json::exception &)
        {
            return false;
        }
    }
} // namespace RLController
