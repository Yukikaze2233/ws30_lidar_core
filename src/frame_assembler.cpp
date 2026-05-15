#include "ws30_lidar/frame_assembler.hpp"

namespace ws30_lidar {
namespace {

constexpr float kCoordinateScaleM = 0.002F;

auto make_point(const PointSample& sample, std::uint64_t timestamp_ms) -> Point {
    return Point{
        .x_m = static_cast<float>(sample.x_raw) * kCoordinateScaleM,
        .y_m = static_cast<float>(sample.y_raw) * kCoordinateScaleM,
        .z_m = static_cast<float>(sample.z_raw) * kCoordinateScaleM,
        .intensity = sample.intensity,
        .ring = static_cast<std::uint16_t>(sample.col / 4),
        .time_offset_s = static_cast<float>(timestamp_ms) * 0.001F,
        .row = sample.row,
        .col = sample.col,
    };
}

} // namespace

auto FrameAssembler::push(const PointsPacket& packet) -> std::optional<PointFrame> {
    if (packet.label == 0x00 || !active_) {
        reset();
        active_ = true;
        building_.timestamp_ms = packet.timestamp_ms;
    }

    append_packet(packet);

    if (packet.label != 0x02) return std::nullopt;

    active_ = false;
    auto completed = std::move(building_);
    building_ = {};
    return completed;
}

auto FrameAssembler::reset() -> void {
    active_ = false;
    building_ = {};
}

auto FrameAssembler::append_packet(const PointsPacket& packet) -> void {
    building_.points.reserve(building_.points.size() + packet.samples.size());
    for (const auto& sample : packet.samples) {
        building_.points.push_back(make_point(sample, packet.timestamp_ms));
    }
}

} // namespace ws30_lidar
