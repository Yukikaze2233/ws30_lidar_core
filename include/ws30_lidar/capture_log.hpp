#pragma once

#include <cstddef>
#include <expected>
#include <filesystem>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "ws30_lidar/types.hpp"

namespace ws30_lidar {

struct RawLogEntry {
    StreamKind stream = StreamKind::points;
    std::uint64_t capture_unix_ns = 0;
    std::vector<std::byte> payload{};
};

class RawLogWriter {
public:
    static auto open(const std::filesystem::path& path)
        -> std::expected<RawLogWriter, std::string>;

    auto append(StreamKind stream,
                std::uint64_t capture_unix_ns,
                std::span<const std::byte> payload)
        -> std::expected<void, std::string>;

private:
    explicit RawLogWriter(std::ofstream stream) noexcept;

    std::ofstream stream_;
};

class RawLogReader {
public:
    static auto open(const std::filesystem::path& path)
        -> std::expected<RawLogReader, std::string>;

    auto read_next()
        -> std::expected<std::optional<RawLogEntry>, std::string>;

private:
    explicit RawLogReader(std::ifstream stream) noexcept;

    std::ifstream stream_;
};

} // namespace ws30_lidar
