#include <array>
#include <cstdint>
#include <cstring>
#include <print>

#include "ws30_lidar/packet_parser.hpp"
#include "test_utils.hpp"

namespace {

using namespace ws30_lidar;

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
#pragma pack(pop)

void test_parse_points_packet() {
    RawPointsPacket raw{};
    raw.data_type[0] = 0x5A;
    raw.data_type[1] = 0xA5;
    raw.timestamp_ms = 123456;
    raw.label = 2;
    raw.row[0] = 7;
    raw.col[0] = 19;
    raw.intensity[0] = 42;
    raw.point_x[0] = 100;
    raw.point_y[0] = -200;
    raw.point_z[0] = 300;

    std::array<std::byte, sizeof(raw)> bytes{};
    std::memcpy(bytes.data(), &raw, sizeof(raw));

    const auto parsed = PacketParser::parse(bytes);
    ws30_lidar_tests::require(parsed.has_value(), "points packet should parse");
    ws30_lidar_tests::require(parsed->kind == PacketKind::points, "points kind mismatch");
    const auto& packet = std::get<PointsPacket>(parsed->data);
    ws30_lidar_tests::require(packet.timestamp_ms == 123456, "points timestamp mismatch");
    ws30_lidar_tests::require(packet.label == 2, "points label mismatch");
    ws30_lidar_tests::require(packet.samples[0].row == 7, "points row mismatch");
    ws30_lidar_tests::require(packet.samples[0].col == 19, "points col mismatch");
    ws30_lidar_tests::require(packet.samples[0].intensity == 42, "points intensity mismatch");
    ws30_lidar_tests::require(packet.samples[0].x_raw == 100, "points x mismatch");
    ws30_lidar_tests::require(packet.samples[0].y_raw == -200, "points y mismatch");
    ws30_lidar_tests::require(packet.samples[0].z_raw == 300, "points z mismatch");
}

void test_parse_imu_packet() {
    RawImuPacket raw{};
    raw.data_type[0] = 0x1A;
    raw.data_type[1] = 0xA1;
    raw.timestamp_ms = 654321;
    raw.gyro_x = 1.0F;
    raw.gyro_y = 2.0F;
    raw.gyro_z = 3.0F;
    raw.acc_x = 4.0F;
    raw.acc_y = 5.0F;
    raw.acc_z = 6.0F;

    std::array<std::byte, sizeof(raw)> bytes{};
    std::memcpy(bytes.data(), &raw, sizeof(raw));

    const auto parsed = PacketParser::parse(bytes);
    ws30_lidar_tests::require(parsed.has_value(), "imu packet should parse");
    ws30_lidar_tests::require(parsed->kind == PacketKind::imu, "imu kind mismatch");
    const auto& imu = std::get<ImuPacket>(parsed->data);
    ws30_lidar_tests::require(imu.timestamp_ms == 654321, "imu timestamp mismatch");
    ws30_lidar_tests::require_near(imu.gyro_x, 1.0F, 1e-5F, "imu gyro_x mismatch");
    ws30_lidar_tests::require_near(imu.acc_z, 6.0F, 1e-5F, "imu acc_z mismatch");
}

void test_parse_unknown_packet() {
    std::array<std::byte, 4> bytes{
        std::byte{0xAA}, std::byte{0x55}, std::byte{0x00}, std::byte{0x01}
    };
    const auto parsed = PacketParser::parse(bytes);
    ws30_lidar_tests::require(parsed.has_value(), "unknown packet should still parse");
    ws30_lidar_tests::require(parsed->kind == PacketKind::unknown, "unknown kind mismatch");
}

} // namespace

int main() {
    test_parse_points_packet();
    test_parse_imu_packet();
    test_parse_unknown_packet();
    std::println("packet_parser_test: PASSED");
    return 0;
}
