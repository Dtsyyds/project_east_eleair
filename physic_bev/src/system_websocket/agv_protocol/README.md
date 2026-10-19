# agv_protocol

## 📡 功能描述
`agv_protocol` 是 AGV 控制系统的通信协议库。它基于 **Boost.Beast** 和 **JsonCpp** 开发，提供了统一的 WebSocket 通信基础设施和 JSON 消息解析能力。

## 🛠️ 核心功能
- **JSON 命令解析**: 定义了标准的通信协议格式，包括 Header 和 Payload。
- **WebSocket 客户端**: 支持异步连接、自动重连和 TLS（可选）。
- **WebSocket 服务端**: 支持多客户端并发连接，用于本地调试或小规模部署。

## 📂 项目结构
- `include/agv_protocol/`: 协议头文件
- `src/`: 协议实现代码

## ⚙️ 依赖
- `rclcpp`
- `jsoncpp`
- `boost` (Beast, Asio)

---
*更多详细协议定义请参考项目根目录下的 [README.md](../../README.md)。*
