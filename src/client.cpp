#include "ws30_lidar/client.hpp"

#include <array>

namespace ws30_lidar {

Client::Client(ClientConfig config)
    : config_(std::move(config)) {}

auto Client::open() -> std::expected<void, std::string> {
    auto points = open_socket(config_.points_port);
    if (!points) return std::unexpected(points.error());
    auto imu = open_socket(config_.imu_port);
    if (!imu) return std::unexpected(imu.error());
    auto status = open_socket(config_.status_port);
    if (!status) return std::unexpected(status.error());

    points_socket_.emplace(std::move(*points));
    imu_socket_.emplace(std::move(*imu));
    status_socket_.emplace(std::move(*status));
    assembler_.reset();
    return {};
}

auto Client::close() noexcept -> void {
    if (points_socket_) points_socket_->close();
    if (imu_socket_) imu_socket_->close();
    if (status_socket_) status_socket_->close();
    points_socket_.reset();
    imu_socket_.reset();
    status_socket_.reset();
    assembler_.reset();
}

auto Client::request_points_stream(bool enabled) -> std::expected<void, std::string> {
    if (!points_socket_) return std::unexpected("points socket is not open");
    return points_socket_->send_text(enabled ? "hello,points" : "stop,points");
}

auto Client::request_imu_stream(bool enabled) -> std::expected<void, std::string> {
    if (!imu_socket_) return std::unexpected("IMU socket is not open");
    return imu_socket_->send_text(enabled ? "hello,imu" : "stop,imu");
}

auto Client::request_serial_number() -> std::expected<void, std::string> {
    if (!status_socket_) return std::unexpected("status socket is not open");
    return status_socket_->send_text("sn");
}

auto Client::poll_points_frame() -> std::expected<std::optional<PointFrame>, std::string> {
    if (!points_socket_) return std::unexpected("points socket is not open");

    while (true) {
        auto packet = receive_packet(*points_socket_);
        if (!packet) return std::unexpected(packet.error());
        if (!packet->has_value()) return std::optional<PointFrame>{};
        if ((*packet)->kind != PacketKind::points) continue;
        const auto& points = std::get<PointsPacket>((*packet)->data);
        if (auto completed = assembler_.push(points); completed) return completed;
    }
}

auto Client::poll_imu_sample() -> std::expected<std::optional<ImuSample>, std::string> {
    if (!imu_socket_) return std::unexpected("IMU socket is not open");
    auto packet = receive_packet(*imu_socket_);
    if (!packet) return std::unexpected(packet.error());
    if (!packet->has_value()) return std::optional<ImuSample>{};
    if ((*packet)->kind != PacketKind::imu) return std::optional<ImuSample>{};

    const auto& imu = std::get<ImuPacket>((*packet)->data);
    return ImuSample{
        .timestamp_ms = imu.timestamp_ms,
        .gyro_x = imu.gyro_x,
        .gyro_y = imu.gyro_y,
        .gyro_z = imu.gyro_z,
        .acc_x = imu.acc_x,
        .acc_y = imu.acc_y,
        .acc_z = imu.acc_z,
    };
}

auto Client::poll_device_info() -> std::expected<std::optional<DeviceInfo>, std::string> {
    if (!status_socket_) return std::unexpected("status socket is not open");
    auto packet = receive_packet(*status_socket_);
    if (!packet) return std::unexpected(packet.error());
    if (!packet->has_value()) return std::optional<DeviceInfo>{};

    if ((*packet)->kind == PacketKind::serial_number) {
        const auto& sn = std::get<SerialNumberPacket>((*packet)->data);
        return DeviceInfo{.serial_number = sn.serial_number};
    }
    if ((*packet)->kind == PacketKind::startup_status) {
        const auto& status = std::get<StartupStatusPacket>((*packet)->data);
        return DeviceInfo{.connected = status.connected};
    }

    return std::optional<DeviceInfo>{};
}

auto Client::is_open() const noexcept -> bool {
    return points_socket_.has_value() && imu_socket_.has_value() && status_socket_.has_value();
}

auto Client::open_socket(std::uint16_t port) const -> std::expected<UdpSocket, std::string> {
    return UdpSocket::open(UdpConfig{
        .remote_address = config_.device_ip,
        .remote_port = port,
        .receive_timeout_ms = config_.receive_timeout_ms,
    });
}

auto Client::receive_packet(UdpSocket& socket) const
    -> std::expected<std::optional<Packet>, std::string> {
    std::array<std::byte, 4096> buffer{};
    const auto received = socket.receive(buffer);
    if (!received) {
        if (received.error().contains("Resource temporarily unavailable")) {
            return std::optional<Packet>{};
        }
        return std::unexpected(received.error());
    }

    auto parsed = PacketParser::parse(std::span<const std::byte>(buffer.data(), *received));
    if (!parsed) return std::unexpected(parsed.error());
    return std::optional<Packet>(std::move(parsed.value()));
}

} // namespace ws30_lidar
