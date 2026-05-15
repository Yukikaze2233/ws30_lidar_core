#include <print>

#include "ws30_lidar/frame_assembler.hpp"
#include "test_utils.hpp"

namespace {

using namespace ws30_lidar;

auto make_points_packet(std::uint8_t label,
                        std::uint64_t timestamp_ms,
                        std::uint16_t row_seed,
                        std::int16_t x_seed) -> PointsPacket {
    PointsPacket packet;
    packet.label = label;
    packet.timestamp_ms = timestamp_ms;
    for (std::size_t i = 0; i < packet.samples.size(); ++i) {
        packet.samples[i] = PointSample{
            .row = static_cast<std::uint16_t>(row_seed + i),
            .col = static_cast<std::uint16_t>(i),
            .intensity = static_cast<std::uint8_t>(10 + (i % 10)),
            .x_raw = static_cast<std::int16_t>(x_seed + static_cast<std::int16_t>(i)),
            .y_raw = static_cast<std::int16_t>(2 * static_cast<std::int16_t>(i)),
            .z_raw = static_cast<std::int16_t>(3 * static_cast<std::int16_t>(i)),
        };
    }
    return packet;
}

void test_frame_completes_on_end_label() {
    FrameAssembler assembler;
    const auto first = make_points_packet(0x00, 1000, 0, 100);
    const auto middle = make_points_packet(0x01, 1001, 1000, 200);
    const auto end = make_points_packet(0x02, 1002, 2000, 300);

    ws30_lidar_tests::require(!assembler.push(first).has_value(), "start packet should not complete frame");
    ws30_lidar_tests::require(!assembler.push(middle).has_value(), "middle packet should not complete frame");
    const auto frame = assembler.push(end);
    ws30_lidar_tests::require(frame.has_value(), "end packet should complete frame");
    ws30_lidar_tests::require(frame->timestamp_ms == 1000, "frame timestamp should come from start packet");
    ws30_lidar_tests::require(frame->points.size() == 3 * kPointsPerPacket, "frame point count mismatch");
    ws30_lidar_tests::require(frame->points.front().row == 0, "first row mismatch");
    ws30_lidar_tests::require(frame->points.back().row == 2119, "last row mismatch");
    ws30_lidar_tests::require_near(frame->points.front().x_m, 0.2F, 1e-5F, "first x scale mismatch");
    ws30_lidar_tests::require(frame->points[120].row == 1000, "second packet offset mismatch");
}

void test_new_start_resets_previous_partial_frame() {
    FrameAssembler assembler;
    const auto stale = make_points_packet(0x01, 1000, 0, 10);
    const auto fresh_start = make_points_packet(0x00, 2000, 500, 20);
    const auto fresh_end = make_points_packet(0x02, 2001, 1000, 30);

    ws30_lidar_tests::require(!assembler.push(stale).has_value(), "stale packet should start implicit partial frame");
    ws30_lidar_tests::require(!assembler.push(fresh_start).has_value(), "fresh start should reset partial frame");
    const auto frame = assembler.push(fresh_end);
    ws30_lidar_tests::require(frame.has_value(), "fresh end should complete frame");
    ws30_lidar_tests::require(frame->timestamp_ms == 2000, "fresh frame timestamp mismatch");
    ws30_lidar_tests::require(frame->points.size() == 2 * kPointsPerPacket, "reset should drop stale points");
    ws30_lidar_tests::require(frame->points.front().row == 500, "fresh frame first row mismatch");
}

} // namespace

int main() {
    test_frame_completes_on_end_label();
    test_new_start_resets_previous_partial_frame();
    std::println("frame_assembler_test: PASSED");
    return 0;
}
