#include <array>
#include <print>

#include "ws30_lidar/capture_log.hpp"
#include "test_utils.hpp"

int main() {
    using namespace ws30_lidar;
    using namespace ws30_lidar_tests;

    const auto path = make_temp_path("raw_log_test");
    {
        auto writer = RawLogWriter::open(path);
        require(writer.has_value(), "raw log writer should open");
        std::array<std::byte, 4> payload{
            std::byte{0x5A}, std::byte{0xA5}, std::byte{0x01}, std::byte{0x02}
        };
        auto append = writer->append(StreamKind::points, 123456789ULL, payload);
        require(append.has_value(), "raw log append should succeed");
    }

    auto reader = RawLogReader::open(path);
    require(reader.has_value(), "raw log reader should open");
    auto entry = reader->read_next();
    require(entry.has_value(), "raw log should read first entry");
    require(entry->has_value(), "raw log first entry should exist");
    require((*entry)->stream == StreamKind::points, "raw log stream mismatch");
    require((*entry)->capture_unix_ns == 123456789ULL, "raw log timestamp mismatch");
    require((*entry)->payload.size() == 4, "raw log payload size mismatch");
    require(static_cast<std::uint8_t>((*entry)->payload[0]) == 0x5A, "raw log payload byte mismatch");

    auto eof = reader->read_next();
    require(eof.has_value(), "raw log eof read should succeed");
    require(!eof->has_value(), "raw log should hit eof");

    std::println("capture_log_test: PASSED");
    return 0;
}
