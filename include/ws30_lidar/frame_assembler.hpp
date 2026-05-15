#pragma once

#include <optional>

#include "ws30_lidar/types.hpp"

namespace ws30_lidar {

class FrameAssembler {
public:
    auto push(const PointsPacket& packet) -> std::optional<PointFrame>;
    auto reset() -> void;

private:
    auto append_packet(const PointsPacket& packet) -> void;

    PointFrame building_{};
    bool active_ = false;
};

} // namespace ws30_lidar
