# ws30_lidar_core

Standalone C++ library for WS_30PCD_ET3 solid-state lidar.

Zero ROS dependency. Zero OpenCV dependency. Pure C++23 + POSIX sockets.

## Features

- UDP transport
- Protocol parser (points / IMU / SN / status)
- Multi-packet frame assembly
- Unified client API
- Raw packet recording and replay
- ASCII PCD export

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

## API Example

```cpp
#include <ws30_lidar/client.hpp>

auto client = ws30_lidar::Client({
    .device_ip = "192.168.137.200",
});

if (auto result = client.open(); !result) {
    std::println(stderr, "open failed: {}", result.error());
    return 1;
}

client.request_points_stream(true);

while (true) {
    auto frame = client.poll_points_frame();
    if (!frame) { /* handle error */ }
    if (frame->has_value()) {
        std::println("received {} points", frame->value().points.size());
    }
}
```

## License

Apache-2.0
