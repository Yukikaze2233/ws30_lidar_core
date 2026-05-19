# ws30_lidar_core

`ws30_lidar_core` is a standalone C++ library for the `WS_30PCD_ET3` solid-state lidar.

It is deliberately kept:

- ROS-free
- OpenCV-free
- UI-free
- focused on raw transport / parsing / frame assembly / capture utilities

The target use cases are:

1. verify that WS30 raw UDP data is correct
2. debug protocol / parser / frame assembly independently from ROS2 or UI
3. provide a clean reusable core for CLI tools, bridge nodes, and replay workflows

## What This Library Provides

- UDP transport wrapper (`UdpSocket`)
- protocol parser for:
  - point packets
  - IMU packets
  - serial number packets
  - startup status packets
- multi-packet point cloud frame assembly (`FrameAssembler`)
- unified high-level polling client (`Client`)
- raw packet log writer / reader (`RawLogWriter`, `RawLogReader`)
- ASCII PCD export (`write_frame_as_pcd`)

## What This Library Does Not Do

- no ROS2 messages
- no PointCloud2 publishing
- no Foxglove / RViz logic
- no camera fusion
- no calibration / `/tf`
- no OpenCV-based visualization

If you need ROS2 integration, put that in a separate bridge package and keep this repository as the protocol/runtime core.

## Repository Layout

```text
include/ws30_lidar/
  udp.hpp            # UDP socket wrapper
  packet_parser.hpp  # parse raw datagrams -> Packet
  frame_assembler.hpp# PointsPacket stream -> PointFrame
  client.hpp         # high-level client API
  capture_log.hpp    # raw packet recording / replay
  pcd_writer.hpp     # PointFrame -> ASCII PCD
  types.hpp          # shared packet / frame / point types

src/
  *.cpp              # implementation

tests/
  *_test.cpp         # unit tests for parser / assembler / rawlog / PCD
```

## Build

Requirements:

- CMake >= 3.16
- C++23 compiler
- POSIX sockets environment

Build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

The library target is:

```text
ws30_lidar_core
```

## Run Tests

```bash
ctest --test-dir build --output-on-failure
```

Current test coverage includes:

- packet parsing
- frame assembly
- raw log read/write
- PCD export

## Public API Overview

### `ws30_lidar::Client`

High-level polling API that owns three UDP sockets:

- points stream (`1001` by default)
- IMU stream (`1002` by default)
- status / serial number (`1003` by default)

Config:

```cpp
struct ClientConfig {
    std::string device_ip = "192.168.137.200";
    std::uint16_t points_port = 1001;
    std::uint16_t imu_port = 1002;
    std::uint16_t status_port = 1003;
    int receive_timeout_ms = 50;
};
```

Key methods:

- `open()` / `close()`
- `request_points_stream(bool)`
- `request_imu_stream(bool)`
- `request_serial_number()`
- `poll_points_frame()`
- `poll_imu_sample()`
- `poll_device_info()`

### Return Semantics

The client APIs use `std::expected<std::optional<T>, std::string>` for poll methods.

Interpretation:

- `expected` error -> actual failure, stop and inspect error string
- `expected` success + `std::nullopt` -> no complete data yet / timeout path
- `expected` success + `value` -> valid parsed sample/frame

This lets callers distinguish clean timeout/no-data cases from hard parser/socket errors.

## Live Device Workflow

Typical online workflow:

```cpp
#include <print>
#include <ws30_lidar/client.hpp>

int main() {
    ws30_lidar::Client client({
        .device_ip = "192.168.137.200",
        .receive_timeout_ms = 50,
    });

    if (auto result = client.open(); !result) {
        std::println(stderr, "open failed: {}", result.error());
        return 1;
    }

    if (auto result = client.request_points_stream(true); !result) {
        std::println(stderr, "request points failed: {}", result.error());
        return 1;
    }

    while (true) {
        auto frame = client.poll_points_frame();
        if (!frame) {
            std::println(stderr, "poll failed: {}", frame.error());
            break;
        }
        if (!frame->has_value()) {
            continue; // timeout / no complete frame yet
        }

        std::println("received frame ts={}ms points={}",
                     frame->value().timestamp_ms,
                     frame->value().points.size());
    }

    client.close();
    return 0;
}
```

Typical optional side flows:

- call `request_imu_stream(true)` if IMU is needed
- call `request_serial_number()` once after `open()` if device identity is needed

## Raw Packet Capture Workflow

Use raw logs when you want to separate:

- device transport issues
- parser issues
- bridge / visualization issues

### Record Raw Datagrams

```cpp
#include <chrono>
#include <print>
#include <ws30_lidar/capture_log.hpp>
#include <ws30_lidar/client.hpp>

// Example idea:
// - read bytes from your UDP receive path
// - append them with stream kind + capture timestamp
```

The raw log format stores:

- stream kind (`points` / `imu` / `status`)
- capture timestamp in unix ns
- raw UDP payload bytes

This is the format consumed by `RawLogReader` later.

### Replay Raw Logs

```cpp
#include <print>
#include <ws30_lidar/capture_log.hpp>
#include <ws30_lidar/packet_parser.hpp>

int main() {
    auto reader = ws30_lidar::RawLogReader::open("/tmp/ws30.rawlog");
    if (!reader) {
        std::println(stderr, "open log failed: {}", reader.error());
        return 1;
    }

    while (true) {
        auto entry = reader->read_next();
        if (!entry) {
            std::println(stderr, "read log failed: {}", entry.error());
            return 1;
        }
        if (!entry->has_value()) break;

        auto parsed = ws30_lidar::PacketParser::parse(entry->value().payload);
        if (!parsed) {
            std::println(stderr, "parse failed: {}", parsed.error());
            continue;
        }

        std::println("stream={} kind={}",
                     ws30_lidar::stream_kind_name(entry->value().stream),
                     ws30_lidar::packet_kind_name(parsed->kind));
    }
}
```

## Frame Assembly Workflow

If you are not using `Client`, you can assemble point frames manually:

```cpp
#include <ws30_lidar/frame_assembler.hpp>
#include <ws30_lidar/packet_parser.hpp>

ws30_lidar::FrameAssembler assembler;

// for each raw UDP datagram:
auto parsed = ws30_lidar::PacketParser::parse(payload);
if (parsed && parsed->kind == ws30_lidar::PacketKind::points) {
    const auto& packet = std::get<ws30_lidar::PointsPacket>(parsed->data);
    if (auto frame = assembler.push(packet); frame) {
        // got one complete PointFrame
    }
}
```

`FrameAssembler::push()` returns `std::optional<PointFrame>`:

- `std::nullopt` -> still assembling
- `PointFrame` -> a full frame is complete

## Export PCD

Once you have a `PointFrame`, export it as ASCII PCD:

```cpp
#include <print>
#include <ws30_lidar/pcd_writer.hpp>

auto result = ws30_lidar::write_frame_as_pcd("/tmp/frame_0001.pcd", frame);
if (!result) {
    std::println(stderr, "write pcd failed: {}", result.error());
}
```

This is useful for:

- CloudCompare / PCL offline inspection
- regression fixtures
- validating parser / assembly output without a live device

## Important Types

### `PointFrame`

Represents one assembled point cloud frame:

```cpp
struct PointFrame {
    std::uint64_t timestamp_ms;
    std::vector<Point> points;
};
```

### `Point`

Each point already contains metric coordinates and metadata:

```cpp
struct Point {
    float x_m;
    float y_m;
    float z_m;
    std::uint8_t intensity;
    std::uint16_t ring;
    float time_offset_s;
    std::uint16_t row;
    std::uint16_t col;
};
```

Documented meaning from current implementation:

- `x_m / y_m / z_m` are metric coordinates from parsed WS30 payload
- `row / col` preserve the sensor grid position carried by the protocol
- `timestamp_ms` is the frame timestamp derived from point packets

## Suggested Debug Order

When something is wrong, debug in this order:

1. `UdpSocket` receive path
2. `PacketParser::parse()` correctness
3. `FrameAssembler::push()` completion behavior
4. `Client` polling loop behavior
5. raw-log replay consistency
6. PCD export inspection

Do not start from ROS2 or UI if the raw datagrams have not been validated.

## Common Notes

- `poll_points_frame()` loops internally until it either:
  - gets a complete frame
  - hits timeout/no-data
  - hits a real error
- timeout/no-data is not treated as a fatal error
- `request_points_stream(true)` sends the WS30 text command `hello,points`
- `request_points_stream(false)` sends `stop,points`
- `request_imu_stream(true)` sends `hello,imu`
- `request_serial_number()` sends `sn`

## License

Apache-2.0
