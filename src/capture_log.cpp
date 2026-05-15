#include "ws30_lidar/capture_log.hpp"

#include <array>

namespace ws30_lidar {
namespace {

constexpr std::array<char, 8> kMagic{ 'W', 'S', '3', '0', 'L', 'O', 'G', '1' };

#pragma pack(push, 1)
struct RawLogEntryHeader {
    std::uint8_t stream = 0;
    std::uint64_t capture_unix_ns = 0;
    std::uint32_t payload_size = 0;
};
#pragma pack(pop)

} // namespace

RawLogWriter::RawLogWriter(std::ofstream stream) noexcept
    : stream_(std::move(stream)) {}

auto RawLogWriter::open(const std::filesystem::path& path)
    -> std::expected<RawLogWriter, std::string> {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path, std::ios::binary);
    if (!stream) return std::unexpected("failed to open raw log for writing");
    stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
    if (!stream) return std::unexpected("failed to write raw log header");
    return RawLogWriter(std::move(stream));
}

auto RawLogWriter::append(StreamKind stream,
                          std::uint64_t capture_unix_ns,
                          std::span<const std::byte> payload)
    -> std::expected<void, std::string> {
    RawLogEntryHeader header{
        .stream = static_cast<std::uint8_t>(stream),
        .capture_unix_ns = capture_unix_ns,
        .payload_size = static_cast<std::uint32_t>(payload.size()),
    };
    stream_.write(reinterpret_cast<const char*>(&header), sizeof(header));
    stream_.write(reinterpret_cast<const char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    if (!stream_) return std::unexpected("failed to append raw log entry");
    return {};
}

RawLogReader::RawLogReader(std::ifstream stream) noexcept
    : stream_(std::move(stream)) {}

auto RawLogReader::open(const std::filesystem::path& path)
    -> std::expected<RawLogReader, std::string> {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) return std::unexpected("failed to open raw log for reading");

    std::array<char, kMagic.size()> magic{};
    stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (!stream) return std::unexpected("failed to read raw log header");
    if (magic != kMagic) return std::unexpected("invalid raw log header");
    return RawLogReader(std::move(stream));
}

auto RawLogReader::read_next()
    -> std::expected<std::optional<RawLogEntry>, std::string> {
    RawLogEntryHeader header{};
    stream_.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (stream_.eof()) return std::optional<RawLogEntry>{};
    if (!stream_) return std::unexpected("failed to read raw log entry header");

    std::vector<std::byte> payload(header.payload_size);
    stream_.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    if (!stream_) return std::unexpected("failed to read raw log payload");

    return RawLogEntry{
        .stream = static_cast<StreamKind>(header.stream),
        .capture_unix_ns = header.capture_unix_ns,
        .payload = std::move(payload),
    };
}

auto stream_kind_name(StreamKind kind) -> const char* {
    switch (kind) {
    case StreamKind::points: return "points";
    case StreamKind::imu: return "imu";
    case StreamKind::status: return "status";
    }
    return "unknown";
}

} // namespace ws30_lidar
