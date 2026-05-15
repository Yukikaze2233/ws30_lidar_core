#include "ws30_lidar/pcd_writer.hpp"

#include <format>
#include <fstream>

namespace ws30_lidar {

auto write_frame_as_pcd(const std::filesystem::path& path,
                        const PointFrame& frame)
    -> std::expected<void, std::string> {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out) return std::unexpected("failed to open PCD output path");

    out << "# .PCD v0.7 - Point Cloud Data file format\n";
    out << "VERSION 0.7\n";
    out << "FIELDS x y z intensity ring time row col\n";
    out << "SIZE 4 4 4 4 4 4 4 4\n";
    out << "TYPE F F F F F F F F\n";
    out << "COUNT 1 1 1 1 1 1 1 1\n";
    out << std::format("WIDTH {}\n", frame.points.size());
    out << "HEIGHT 1\n";
    out << "VIEWPOINT 0 0 0 1 0 0 0\n";
    out << std::format("POINTS {}\n", frame.points.size());
    out << "DATA ascii\n";
    for (const auto& point : frame.points) {
        out << std::format("{:.6f} {:.6f} {:.6f} {} {} {:.6f} {} {}\n",
                           point.x_m,
                           point.y_m,
                           point.z_m,
                           static_cast<float>(point.intensity),
                           static_cast<float>(point.ring),
                           point.time_offset_s,
                           static_cast<float>(point.row),
                           static_cast<float>(point.col));
    }
    if (!out) return std::unexpected("failed while writing PCD file");
    return {};
}

} // namespace ws30_lidar
