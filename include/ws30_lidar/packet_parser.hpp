#pragma once

#include <cstddef>
#include <expected>
#include <span>
#include <string>

#include "ws30_lidar/types.hpp"

namespace ws30_lidar {

class PacketParser {
public:
    static auto parse(std::span<const std::byte> payload)
        -> std::expected<Packet, std::string>;
};

} // namespace ws30_lidar
