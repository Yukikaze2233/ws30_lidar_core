# ws30_lidar_core

`ws30_lidar_core` 是一个面向 `WS_30PCD_ET3` 纯固态激光雷达的独立 C++ 库。

这个仓库刻意保持为：

- 不依赖 ROS
- 不依赖 OpenCV
- 不包含 UI
- 专注于原始传输、协议解析、组帧与抓包工具

目标使用场景是：

1. 验证 WS30 原始 UDP 数据是否正确
2. 在不依赖 ROS2 / UI 的情况下单独调试协议、解析器和组帧逻辑
3. 为 CLI 工具、bridge 节点和回放工作流提供一个可复用的干净 core

## 提供的能力

- UDP 传输封装（`UdpSocket`）
- 协议解析器，支持：
  - 点云包
  - IMU 包
  - 序列号包
  - 启动状态包
- 多包点云组帧（`FrameAssembler`）
- 统一高层轮询客户端（`Client`）
- 原始数据包日志写入 / 读取（`RawLogWriter`, `RawLogReader`）
- ASCII PCD 导出（`write_frame_as_pcd`）

## 不做的事情

- 不提供 ROS2 消息
- 不发布 PointCloud2
- 不包含 Foxglove / RViz 逻辑
- 不做相机融合
- 不做标定 / `/tf`
- 不做基于 OpenCV 的可视化

如果需要 ROS2 集成，请把它放到单独的 bridge package 中，让这个仓库继续只承担协议/runtime core 的职责。

## 仓库结构

```text
include/ws30_lidar/
  udp.hpp             # UDP socket 封装
  packet_parser.hpp   # 原始 datagram -> Packet
  frame_assembler.hpp # PointsPacket 流 -> PointFrame
  client.hpp          # 高层 client API
  capture_log.hpp     # 原始包录制 / 回放
  pcd_writer.hpp      # PointFrame -> ASCII PCD
  types.hpp           # 公共 packet / frame / point 类型

src/
  *.cpp               # 实现

tests/
  *_test.cpp          # parser / assembler / rawlog / PCD 单元测试
```

## 构建

要求：

- CMake >= 3.16
- 支持 C++23 的编译器
- POSIX sockets 环境

构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

生成的库目标为：

```text
ws30_lidar_core
```

## 运行测试

```bash
ctest --test-dir build --output-on-failure
```

当前测试覆盖：

- 协议解析
- 组帧
- raw log 读写
- PCD 导出

## Public API 概览

### `ws30_lidar::Client`

这是一个高层轮询 API，内部持有 3 个 UDP socket：

- 点云流（默认 `1001`）
- IMU 流（默认 `1002`）
- 状态 / 序列号（默认 `1003`）

配置：

```cpp
struct ClientConfig {
    std::string device_ip = "192.168.137.200";
    std::uint16_t points_port = 1001;
    std::uint16_t imu_port = 1002;
    std::uint16_t status_port = 1003;
    int receive_timeout_ms = 50;
};
```

关键方法：

- `open()` / `close()`
- `request_points_stream(bool)`
- `request_imu_stream(bool)`
- `request_serial_number()`
- `poll_points_frame()`
- `poll_imu_sample()`
- `poll_device_info()`

### 返回语义

`poll_*` 系列接口使用 `std::expected<std::optional<T>, std::string>` 作为返回类型。

含义如下：

- `expected` 为 error -> 真正的失败，应该停下来检查错误字符串
- `expected` 成功 + `std::nullopt` -> 当前还没有完整数据 / 只是 timeout 路径
- `expected` 成功 + `value` -> 得到了有效解析结果或完整帧

这样调用方可以把“正常 timeout / 暂时没数据”和“真正的 socket / parser 错误”区分开。

## 联机工作流

典型联机流程：

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
            continue; // timeout / 尚未组成完整帧
        }

        std::println("received frame ts={}ms points={}",
                     frame->value().timestamp_ms,
                     frame->value().points.size());
    }

    client.close();
    return 0;
}
```

可选的旁路操作：

- 如果需要 IMU，就调用 `request_imu_stream(true)`
- 如果需要设备身份信息，就在 `open()` 后调用一次 `request_serial_number()`

## Raw 包录制工作流

当你想把下面几类问题拆开定位时，应优先录 raw log：

- 设备传输问题
- parser 问题
- bridge / 可视化问题

### 录制原始 datagram

```cpp
#include <chrono>
#include <print>
#include <ws30_lidar/capture_log.hpp>
#include <ws30_lidar/client.hpp>

// 示例思路：
// - 从你的 UDP 接收路径拿到原始字节
// - 连同 stream kind 和采集时间一起 append
```

raw log 格式会保存：

- stream kind（`points` / `imu` / `status`）
- unix ns 级采集时间戳
- 原始 UDP payload 字节串

后续 `RawLogReader` 读取的就是这个格式。

### 回放 raw log

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

## 组帧工作流

如果你不想用 `Client`，也可以手动组点云帧：

```cpp
#include <ws30_lidar/frame_assembler.hpp>
#include <ws30_lidar/packet_parser.hpp>

ws30_lidar::FrameAssembler assembler;

// 对每个原始 UDP datagram：
auto parsed = ws30_lidar::PacketParser::parse(payload);
if (parsed && parsed->kind == ws30_lidar::PacketKind::points) {
    const auto& packet = std::get<ws30_lidar::PointsPacket>(parsed->data);
    if (auto frame = assembler.push(packet); frame) {
        // 得到一帧完整 PointFrame
    }
}
```

`FrameAssembler::push()` 的返回语义：

- `std::nullopt` -> 还在继续组帧
- `PointFrame` -> 已经凑出一帧完整点云

## 导出 PCD

拿到 `PointFrame` 后，可以导出成 ASCII PCD：

```cpp
#include <print>
#include <ws30_lidar/pcd_writer.hpp>

auto result = ws30_lidar::write_frame_as_pcd("/tmp/frame_0001.pcd", frame);
if (!result) {
    std::println(stderr, "write pcd failed: {}", result.error());
}
```

适合用在：

- CloudCompare / PCL 离线检查
- 回归测试样本
- 在无设备情况下验证 parser / assembler 输出

## 关键类型

### `PointFrame`

表示一帧完整组装出来的点云：

```cpp
struct PointFrame {
    std::uint64_t timestamp_ms;
    std::vector<Point> points;
};
```

### `Point`

每个点已经带有米制坐标和元数据：

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

基于当前实现，可按以下方式理解：

- `x_m / y_m / z_m` 是从 WS30 payload 解析出的米制坐标
- `row / col` 保留协议里自带的传感器网格位置
- `timestamp_ms` 是从点云包中抽出的帧时间戳

## 建议的调试顺序

出问题时，建议按下面顺序排查：

1. `UdpSocket` 接收路径
2. `PacketParser::parse()` 解析正确性
3. `FrameAssembler::push()` 组帧完成条件
4. `Client` 轮询循环行为
5. raw-log 回放一致性
6. PCD 导出结果

如果原始 datagram 还没验证正确，就不要直接从 ROS2 或 UI 往上查。

## 常见说明

- `poll_points_frame()` 内部会持续循环，直到出现以下三种结果之一：
  - 得到完整帧
  - timeout / 暂时没数据
  - 真正错误
- timeout / no-data 不视为致命错误
- `request_points_stream(true)` 发送的文本命令是 `hello,points`
- `request_points_stream(false)` 发送的文本命令是 `stop,points`
- `request_imu_stream(true)` 发送的文本命令是 `hello,imu`
- `request_serial_number()` 发送的文本命令是 `sn`

## 许可证

Apache-2.0
