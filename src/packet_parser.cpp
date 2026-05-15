#include "ws30_lidar/packet_parser.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace ws30_lidar {
namespace {

#pragma pack(push, 1)
struct RawPointsPacket {
    std::uint8_t data_type[2];
    std::uint64_t timestamp_ms;
    std::uint8_t label;
    std::uint16_t row[kPointsPerPacket];
    std::uint16_t col[kPointsPerPacket];
    std::uint8_t intensity[kPointsPerPacket];
    std::int16_t point_x[kPointsPerPacket];
    std::int16_t point_y[kPointsPerPacket];
    std::int16_t point_z[kPointsPerPacket];
};

struct RawImuPacket {
    std::uint8_t data_type[2];
    std::uint64_t timestamp_ms;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
};

struct RawStatusPacket {
    std::uint8_t data_type[2];
    char sn_data[64];
    bool is_connected;
};
#pragma pack(pop)

template <typename TRaw>
auto copy_raw(std::span<const std::byte> payload) -> TRaw {
    TRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(TRaw));
    return raw;
}

auto trim_c_string(const char* value, std::size_t size) -> std::string {
    const auto* end = std::find(value, value + size, '\0');
    return std::string(value, end);
}

} // namespace

auto packet_kind_name(PacketKind kind) -> const char* {
    switch (kind) {
    case PacketKind::points: return "points";
    case PacketKind::imu: return "imu";
    case PacketKind::serial_number: return "serial_number";
    case PacketKind::startup_status: return "startup_status";
    case PacketKind::unknown: return "unknown";
    }
    return "unknown";
}

auto PacketParser::parse(std::span<const std::byte> payload)
    -> std::expected<Packet, std::string> {
    if (payload.size() < 2) {
        return std::unexpected("packet too short for header");
    }

    const auto header0 = static_cast<std::uint8_t>(payload[0]);
    const auto header1 = static_cast<std::uint8_t>(payload[1]);

    if (header0 == 0x5A && header1 == 0xA5) {
        if (payload.size() != sizeof(RawPointsPacket)) {
            return std::unexpected("points packet size mismatch");
        }
        const auto raw = copy_raw<RawPointsPacket>(payload);
        PointsPacket points;
        points.timestamp_ms = raw.timestamp_ms;
        points.label = raw.label;
        for (std::size_t i = 0; i < kPointsPerPacket; ++i) {
            points.samples[i] = PointSample{
                .row = raw.row[i],
                .col = raw.col[i],
                .intensity = raw.intensity[i],
                .x_raw = raw.point_x[i],
                .y_raw = raw.point_y[i],
                .z_raw = raw.point_z[i],
            };
        }
        return Packet{
            .kind = PacketKind::points,
            .data = std::move(points),
        };
    }

    if (header0 == 0x1A && header1 == 0xA1) {
        if (payload.size() != sizeof(RawImuPacket)) {
            return std::unexpected("IMU packet size mismatch");
        }
        const auto raw = copy_raw<RawImuPacket>(payload);
        return Packet{
            .kind = PacketKind::imu,
            .data = ImuPacket{
                .timestamp_ms = raw.timestamp_ms,
                .gyro_x = raw.gyro_x,
                .gyro_y = raw.gyro_y,
                .gyro_z = raw.gyro_z,
                .acc_x = raw.acc_x,
                .acc_y = raw.acc_y,
                .acc_z = raw.acc_z,
            },
        };
    }

    if (header0 == 0x2A && header1 == 0xA2) {
        if (payload.size() != sizeof(RawStatusPacket)) {
            return std::unexpected("SN packet size mismatch");
        }
        const auto raw = copy_raw<RawStatusPacket>(payload);
        return Packet{
            .kind = PacketKind::serial_number,
            .data = SerialNumberPacket{
                .serial_number = trim_c_string(raw.sn_data, sizeof(raw.sn_data)),
            },
        };
    }

    if (header0 == 0x3A && header1 == 0xA3) {
        if (payload.size() != sizeof(RawStatusPacket)) {
            return std::unexpected("startup status packet size mismatch");
        }
        const auto raw = copy_raw<RawStatusPacket>(payload);
        return Packet{
            .kind = PacketKind::startup_status,
            .data = StartupStatusPacket{
                .connected = raw.is_connected,
            },
        };
    }

    return Packet{
        .kind = PacketKind::unknown,
        .data = UnknownPacket{
            .header0 = header0,
            .header1 = header1,
        },
    };
}

} // namespace ws30_lidar
