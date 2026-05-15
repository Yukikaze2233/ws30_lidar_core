#pragma once

#include <expected>
#include <optional>
#include <string>

#include "ws30_lidar/frame_assembler.hpp"
#include "ws30_lidar/packet_parser.hpp"
#include "ws30_lidar/udp.hpp"

namespace ws30_lidar {

struct ClientConfig {
    std::string device_ip = "192.168.137.200";
    std::uint16_t points_port = 1001;
    std::uint16_t imu_port = 1002;
    std::uint16_t status_port = 1003;
    int receive_timeout_ms = 50;
};

class Client {
public:
    explicit Client(ClientConfig config);

    auto open() -> std::expected<void, std::string>;
    auto close() noexcept -> void;

    auto request_points_stream(bool enabled) -> std::expected<void, std::string>;
    auto request_imu_stream(bool enabled) -> std::expected<void, std::string>;
    auto request_serial_number() -> std::expected<void, std::string>;

    auto poll_points_frame() -> std::expected<std::optional<PointFrame>, std::string>;
    auto poll_imu_sample() -> std::expected<std::optional<ImuSample>, std::string>;
    auto poll_device_info() -> std::expected<std::optional<DeviceInfo>, std::string>;

    [[nodiscard]] auto is_open() const noexcept -> bool;

private:
    auto open_socket(std::uint16_t port) const -> std::expected<UdpSocket, std::string>;
    auto receive_packet(UdpSocket& socket) const
        -> std::expected<std::optional<Packet>, std::string>;

    ClientConfig config_{};
    std::optional<UdpSocket> points_socket_{};
    std::optional<UdpSocket> imu_socket_{};
    std::optional<UdpSocket> status_socket_{};
    FrameAssembler assembler_{};
};

} // namespace ws30_lidar
