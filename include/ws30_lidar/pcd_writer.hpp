#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include "ws30_lidar/types.hpp"

namespace ws30_lidar {

auto write_frame_as_pcd(const std::filesystem::path& path,
                        const PointFrame& frame)
    -> std::expected<void, std::string>;

} // namespace ws30_lidar
