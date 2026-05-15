#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <variant>

namespace ws30_lidar {

constexpr std::size_t kPointsPerPacket = 120;

enum class PacketKind : std::uint8_t {
    points,
    imu,
    serial_number,
    startup_status,
    unknown,
};

enum class StreamKind : std::uint8_t {
    points = 0,
    imu = 1,
    status = 2,
};

struct PointSample {
    std::uint16_t row = 0;
    std::uint16_t col = 0;
    std::uint8_t intensity = 0;
    std::int16_t x_raw = 0;
    std::int16_t y_raw = 0;
    std::int16_t z_raw = 0;
};

struct Point {
    float x_m = 0.0F;
    float y_m = 0.0F;
    float z_m = 0.0F;
    std::uint8_t intensity = 0;
    std::uint16_t ring = 0;
    float time_offset_s = 0.0F;
    std::uint16_t row = 0;
    std::uint16_t col = 0;
};

struct PointsPacket {
    std::uint64_t timestamp_ms = 0;
    std::uint8_t label = 0;
    std::array<PointSample, kPointsPerPacket> samples{};
};

struct ImuPacket {
    std::uint64_t timestamp_ms = 0;
    float gyro_x = 0.0F;
    float gyro_y = 0.0F;
    float gyro_z = 0.0F;
    float acc_x = 0.0F;
    float acc_y = 0.0F;
    float acc_z = 0.0F;
};

struct SerialNumberPacket {
    std::string serial_number{};
};

struct StartupStatusPacket {
    bool connected = false;
};

struct UnknownPacket {
    std::uint8_t header0 = 0;
    std::uint8_t header1 = 0;
};

using PacketData = std::variant<PointsPacket,
                                ImuPacket,
                                SerialNumberPacket,
                                StartupStatusPacket,
                                UnknownPacket>;

struct Packet {
    PacketKind kind = PacketKind::unknown;
    PacketData data = UnknownPacket{};
};

struct PointFrame {
    std::uint64_t timestamp_ms = 0;
    std::vector<Point> points{};
};

struct ImuSample {
    std::uint64_t timestamp_ms = 0;
    float gyro_x = 0.0F;
    float gyro_y = 0.0F;
    float gyro_z = 0.0F;
    float acc_x = 0.0F;
    float acc_y = 0.0F;
    float acc_z = 0.0F;
};

struct DeviceInfo {
    std::optional<std::string> serial_number{};
    std::optional<bool> connected{};
};

auto packet_kind_name(PacketKind kind) -> const char*;
auto stream_kind_name(StreamKind kind) -> const char*;

} // namespace ws30_lidar
