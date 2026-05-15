#include <print>

#include "ws30_lidar/pcd_writer.hpp"
#include "test_utils.hpp"

int main() {
    using namespace ws30_lidar;
    using namespace ws30_lidar_tests;

    const auto dir = make_temp_dir("pcd_writer_test");
    const auto path = dir / "frame.pcd";
    PointFrame frame;
    frame.timestamp_ms = 42;
    frame.points.push_back(Point{
        .x_m = 1.0F,
        .y_m = 2.0F,
        .z_m = 3.0F,
        .intensity = 4,
        .ring = 5,
        .time_offset_s = 0.25F,
        .row = 6,
        .col = 7,
    });

    auto result = write_frame_as_pcd(path, frame);
    require(result.has_value(), "PCD writer should succeed");
    const auto text = read_text_file(path);
    require_contains(text, "FIELDS x y z intensity ring time row col", "pcd header");
    require_contains(text, "POINTS 1", "pcd point count");
    require_contains(text, "1.000000 2.000000 3.000000 4 5 0.250000 6 7", "pcd point row");

    std::println("pcd_writer_test: PASSED");
    return 0;
}
