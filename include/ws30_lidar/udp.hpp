#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>

namespace ws30_lidar {

struct UdpConfig {
    std::string bind_address = "0.0.0.0";
    std::uint16_t bind_port = 0;
    std::string remote_address{};
    std::uint16_t remote_port = 0;
    int receive_timeout_ms = 50;
};

class UdpSocket {
public:
    UdpSocket() = delete;
    UdpSocket(const UdpSocket&) = delete;
    auto operator=(const UdpSocket&) -> UdpSocket& = delete;
    UdpSocket(UdpSocket&& other) noexcept;
    auto operator=(UdpSocket&& other) noexcept -> UdpSocket&;
    ~UdpSocket() noexcept;

    [[nodiscard]] static auto open(const UdpConfig& config)
        -> std::expected<UdpSocket, std::string>;

    auto receive(std::span<std::byte> buffer)
        -> std::expected<std::size_t, std::string>;

    auto send_text(std::string_view message)
        -> std::expected<void, std::string>;

    auto close() noexcept -> void;

    [[nodiscard]] auto is_open() const noexcept -> bool { return fd_ >= 0; }

private:
    explicit UdpSocket(int fd, UdpConfig config) noexcept;

    int fd_ = -1;
    UdpConfig config_{};
};

} // namespace ws30_lidar
