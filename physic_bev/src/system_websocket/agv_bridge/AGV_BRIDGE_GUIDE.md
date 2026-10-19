# agv_bridge 包技术文档

## 1. 概述

`agv_bridge` 是 AGV ROS2 控制系统中的 **WebSocket 桥接节点**，负责在远程控制端（如 Unity 3D 仿真、Web 前端）与本地 ROS2 系统之间建立通信桥梁。

**核心设计模式：中介者模式 + 组合模式**

- **中介者模式**：`BridgeManager` 作为中心调度器，解耦 WebSocket 协议层与机器人业务逻辑
- **组合模式**：`BaseRobot` 通过组合多个 `DeviceHandler` 来实现复杂机器人的功能

### 架构图

```
远程控制端 (Unity/Web)
       │
       ▼
┌─────────────────────────────────────────────────────┐
│              websocket_bridge_node                    │
│  ┌───────────────────────────────────────────────┐  │
│  │           BridgeManager (中介者)               │  │
│  │                                               │  │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐      │  │
│  │  │ Robot A │  │ Robot B │  │ Robot C │ ...  │  │
│  │  │(BaseRobot)│ │(BaseRobot)│ │(BaseRobot)│    │  │
│  │  └────┬────┘  └────┬────┘  └────┬────┘      │  │
│  │       │             │             │           │  │
│  │  ┌────┴────┐  ┌────┴────┐  ┌────┴────┐      │  │
│  │  │Handler1 │  │Handler1 │  │Handler1 │      │  │
│  │  │Handler2 │  │Handler2 │  │Handler2 │      │  │
│  │  │Handler3 │  │   ...   │  │   ...   │      │  │
│  │  └─────────┘  └─────────┘  └─────────┘      │  │
│  └───────────────────────────────────────────────┘  │
│                                                     │
│  ┌─────────────────┐    ┌──────────────────────┐   │
│  │ WebSocketClient  │    │ ROS2 Publishers /    │   │
│  │ (agv_protocol)   │    │ Subscribers          │   │
│  └─────────────────┘    └──────────────────────┘   │
└─────────────────────────────────────────────────────┘
       │                              │
       ▼                              ▼
  远程服务器                    ROS2 话题/服务
```

---

## 2. 文件清单与功能说明

### 2.1 头文件 (`include/agv_bridge/`)

#### `device_handler.hpp` — 设备处理器接口

**功能**：定义所有设备处理器的抽象基类 `DeviceHandler`。

```cpp
class DeviceHandler {
public:
    virtual void init(rclcpp::Node* node) = 0;           // 初始化 ROS2 资源
    virtual Json::Value handleCommand(const Json::Value& cmd, const Header& header) = 0;  // 处理控制指令
    virtual Json::Value getReport() = 0;                  // 生成状态报告
    virtual bool isOnline() const = 0;                    // 设备在线状态
};
```

**设计要点**：
- 每个设备（底盘、升降机构、机械臂等）实现一个 `DeviceHandler` 子类
- `handleCommand()` 接收 JSON 指令，返回 JSON 响应
- `getReport()` 返回设备当前状态，用于周期性上报
- `isOnline()` 用于自检指令的应答

---

#### `robot.hpp` — 机器人抽象基类与基础实现

**功能**：定义机器人的抽象接口 `Robot` 和基础实现 `BaseRobot`。

**`Robot` 抽象接口**：

| 方法 | 说明 |
|------|------|
| `init(node)` | 初始化机器人，注册 ROS2 话题/服务 |
| `getLaunchCommand()` | 返回启动该机器人的 shell 命令 |
| `handleCommand(msg)` | 处理常规业务指令（如运动控制） |
| `handleCheck(msg)` | 处理自检指令（检查设备在线状态） |
| `generateReports(parser)` | 生成周期性状态报告消息 |
| `getRobotId()` | 返回机器人唯一标识 |

**`BaseRobot` 基础实现**：
- 使用 `std::map<std::string, DeviceHandler::Ptr>` 管理多个设备处理器
- `handleCommand()` 遍历 payload 中的 key，将子指令分发给对应的 handler
- `handleCheck()` 检查每个 handler 的在线状态
- `generateReports()` 收集所有 handler 的状态报告，合并为一条 `status_update` 消息

---

#### `handlers.hpp` — 内置设备处理器集合

**功能**：提供底盘（`ChassisHandler`）和升降机构（`LiftingHandler`）的默认实现。

**`ChassisHandler`** — 底盘控制处理器：

| 功能 | ROS2 话题 | 消息类型 | 方向 |
|------|-----------|----------|------|
| 速度控制 | `/cmd_vel` | `geometry_msgs/Twist` | 发布 |
| 里程计 | `/odom` | `nav_msgs/Odometry` | 订阅 |
| 电量状态 | `/battery_state` | `sensor_msgs/BatteryState` | 订阅 |

支持的指令：
- `move`：速度控制（linear_x, linear_y, angular_z）
- `stop`：紧急停止
- `reset_odom`：里程计归零

**`LiftingHandler`** — 升降机构控制处理器：

| 功能 | ROS2 话题 | 消息类型 | 方向 |
|------|-----------|----------|------|
| 升降控制 | `/lifting_controller/command` | `std_msgs/Float64` | 发布 |
| 升降状态 | `/lifting_controller/state` | `std_msgs/Float64` | 订阅 |

支持的指令：
- `set_height`：设置目标高度（米）
- `stop`：停止升降

---

#### `robots.hpp` — 具体机器人实现

**功能**：定义具体的机器人类型，如 `AgvRobot`。

**`AgvRobot`** 继承自 `BaseRobot`，在构造时自动注册以下设备处理器：

| Handler 名称 | 类型 | 说明 |
|-------------|------|------|
| `chassis` | `ChassisHandler` | 底盘运动控制 |
| `lifting` | `LiftingHandler` | 升降机构控制 |

**`AgvRobotFactory`** — 机器人工厂：
- `createRobot(robot_id)`：根据 robot_id 创建 `AgvRobot` 实例
- `getSupportedTypes()`：返回支持的机器人类型列表 `["agv"]`

---

#### `bridge_manager.hpp` — 桥接管理器

**功能**：系统核心调度器，管理所有机器人实例和 WebSocket 通信。

**主要职责**：

1. **机器人管理**：
   - `addRobot(robot)` / `removeRobot(robot_id)`：动态添加/移除机器人
   - `getRobot(robot_id)`：获取指定机器人实例

2. **消息路由**：
   - `handleMessage(msg)`：根据消息类型分发到对应处理方法
   - `handleCommand(robot_id, msg)` → `robot->handleCommand(msg)`
   - `handleCheck(robot_id, msg)` → `robot->handleCheck(msg)`

3. **状态上报**：
   - `startStatusReport()` / `stopStatusReport()`：启停周期性状态上报
   - 内部定时器按配置间隔收集所有机器人的状态并发送

4. **生命周期管理**：
   - `init(node)`：初始化所有已注册的机器人
   - `shutdown()`：停止上报、断开连接

---

### 2.2 源文件 (`src/`)

#### `bridge_manager.cpp` — 桥接管理器实现

**功能**：实现 `BridgeManager` 的所有方法。

**关键实现细节**：

- **消息路由逻辑**：根据 `msg.header.type` 字段判断消息类型
  - `"command"` → `handleCommand()`：业务控制指令
  - `"check"` → `handleCheck()`：设备自检指令
  - `"launch"` → `handleLaunch()`：启动机器人进程
  - `"stop"` → `handleStop()`：停止机器人进程

- **状态上报**：使用 `rclcpp::TimerBase` 定时触发，遍历所有机器人调用 `generateReports()`，通过 WebSocket 发送

- **进程管理**：`handleLaunch()` 通过 `popen()` 执行 `ros2 launch` 命令启动机器人进程，`handleStop()` 通过 PID 终止进程

---

#### `websocket_bridge_node.cpp` — WebSocket 桥接节点主程序

**功能**：ROS2 节点入口，创建 WebSocket 客户端并连接到远程服务器。

**启动流程**：

1. 创建 ROS2 节点 `websocket_bridge_node`
2. 声明参数：`server_ip`、`server_port`、`report_interval`
3. 创建 `BridgeManager` 实例
4. 创建 `AgvRobotFactory`，注册支持的机器人类型
5. 创建 `WebSocketClient`（来自 agv_protocol 包）
6. 设置消息回调：`client->onMessage()` → `manager->handleMessage()`
7. 连接 WebSocket 服务器
8. 启动状态上报定时器
9. `rclpy.spin(node)` 进入事件循环

**参数说明**：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `server_ip` | string | `"127.0.0.1"` | WebSocket 服务器地址 |
| `server_port` | int | `8765` | WebSocket 服务器端口 |
| `report_interval` | double | `1.0` | 状态上报间隔（秒） |

---

### 2.3 构建配置

#### `CMakeLists.txt`

- 构建共享库 `agv_bridge`（包含 `bridge_manager.cpp`）
- 构建可执行文件 `websocket_bridge_node`
- 依赖：`rclcpp`、`geometry_msgs`、`std_msgs`、`nav_msgs`、`sensor_msgs`、`agv_protocol`、`Boost`、`OpenSSL`
- 导出库和头文件，供其他包使用

#### `package.xml`

- 包名：`agv_bridge`
- 版本：`0.0.1`
- 描述：中介者模式的消息分发包，解耦硬件接口与业务逻辑
- 许可证：Apache License 2.0

---

## 3. 通信协议

### 3.1 消息格式（JSON）

所有消息通过 WebSocket 传输，格式为 JSON：

```json
{
  "header": {
    "robot_id": "agv_001",
    "type": "command",
    "timestamp": 1234567890,
    "msg_id": "uuid-xxxx"
  },
  "payload": {
    "chassis": {
      "command": "move",
      "linear_x": 0.5,
      "linear_y": 0.0,
      "angular_z": 0.0
    },
    "lifting": {
      "command": "set_height",
      "height": 0.3
    }
  }
}
```

### 3.2 消息类型

| type | 方向 | 说明 |
|------|------|------|
| `command` | 远程→本地 | 业务控制指令 |
| `check` | 远程→本地 | 设备自检指令 |
| `launch` | 远程→本地 | 启动机器人进程 |
| `stop` | 远程→本地 | 停止机器人进程 |
| `status_update` | 本地→远程 | 周期性状态上报 |
| `command_response` | 本地→远程 | 指令执行结果 |
| `check_response` | 本地→远程 | 自检结果 |

---

## 4. 如何添加新设备

### 4.1 场景说明

假设你有一个现有的机器人（如 `AgvRobot`），需要添加一个新的设备模块（例如：机械臂控制器 `ArmHandler`）。

### 4.2 步骤

#### 第一步：创建 DeviceHandler 子类

在 `include/agv_bridge/handlers.hpp` 中添加新的处理器类：

```cpp
/**
 * @brief 机械臂控制处理器
 */
class ArmHandler : public DeviceHandler {
public:
    ArmHandler(const std::string& arm_topic_prefix = "/arm")
        : arm_topic_prefix_(arm_topic_prefix) {}

    void init(rclcpp::Node* node) override {
        node_ = node;

        // 创建发布者：发送关节目标位置
        joint_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            arm_topic_prefix_ + "/joint_commands", 10);

        // 创建订阅者：接收关节状态
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            arm_topic_prefix_ + "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_joint_state_ = *msg;
                has_state_ = true;
            });

        RCLCPP_INFO(node_->get_logger(), "ArmHandler 初始化完成 [%s]", arm_topic_prefix_.c_str());
    }

    Json::Value handleCommand(const Json::Value& cmd, 
                              const agv_protocol::Header& header) override {
        Json::Value result;
        std::string command = cmd["command"].asString();

        if (command == "move_joints") {
            // 发送关节位置指令
            auto msg = std_msgs::msg::Float64MultiArray();
            for (const auto& val : cmd["positions"]) {
                msg.data.push_back(val.asDouble());
            }
            joint_cmd_pub_->publish(msg);
            result["status"] = "ok";
            result["command"] = "move_joints";

        } else if (command == "stop") {
            // 停止机械臂
            auto msg = std_msgs::msg::Float64MultiArray();
            // 发送停止指令...
            result["status"] = "ok";
            result["command"] = "stop";

        } else {
            result["status"] = "error";
            result["message"] = "Unknown command: " + command;
        }

        return result;
    }

    Json::Value getReport() override {
        std::lock_guard<std::mutex> lock(mutex_);
        Json::Value report;
        if (has_state_) {
            report["joint_names"] = Json::Value(Json::arrayValue);
            for (const auto& name : latest_joint_state_.name) {
                report["joint_names"].append(name);
            }
            report["joint_positions"] = Json::Value(Json::arrayValue);
            for (const auto& pos : latest_joint_state_.position) {
                report["joint_positions"].append(pos);
            }
        }
        return report;
    }

    bool isOnline() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_state_;  // 收到过状态就认为在线
    }

private:
    std::string arm_topic_prefix_;
    rclcpp::Node* node_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState latest_joint_state_;
    bool has_state_ = false;
    mutable std::mutex mutex_;
};
```

#### 第二步：注册到机器人实例

在 `robots.hpp` 的 `AgvRobot` 构造函数中添加新的 handler：

```cpp
class AgvRobot : public BaseRobot {
public:
    AgvRobot(const std::string& robot_id) : BaseRobot(robot_id) {
        // 已有的 handler
        addHandler("chassis", std::make_shared<ChassisHandler>());
        addHandler("lifting", std::make_shared<LiftingHandler>());

        // ★ 新增：机械臂 handler
        addHandler("arm", std::make_shared<ArmHandler>("/arm"));
    }

    std::string getLaunchCommand() override {
        // 如果需要，更新启动命令以包含机械臂节点
        return "ros2 launch agv_bringup agv.launch.xml";
    }
};
```

#### 第三步：更新 CMakeLists.txt（如需要）

如果新 handler 使用了新的消息类型，需要在 `CMakeLists.txt` 中添加依赖：

```cmake
find_package(sensor_msgs REQUIRED)  # 如果还没有

ament_target_dependencies(${PROJECT_NAME}
  # ... 已有依赖 ...
  sensor_msgs  # 新增
)
```

#### 第四步：测试

发送以下 JSON 消息测试新设备：

```json
// 控制指令
{
  "header": {
    "robot_id": "agv_001",
    "type": "command"
  },
  "payload": {
    "arm": {
      "command": "move_joints",
      "positions": [0.0, -1.57, 1.57, 0.0, -1.57, 0.0]
    }
  }
}

// 自检指令
{
  "header": {
    "robot_id": "agv_001",
    "type": "check"
  },
  "payload": {
    "arm": {
      "command": "is_online"
    }
  }
}
```

---

## 5. 如何添加新机器人

### 5.1 场景说明

假设你需要支持一种全新的机器人类型（例如：巡检机器人 `PatrolRobot`），它有不同的设备组合。

### 5.2 步骤

#### 第一步：创建新的 DeviceHandler（如需要）

如果新机器人有全新的设备类型，按第 4 节的方法创建对应的 `DeviceHandler` 子类。

#### 第二步：创建新的 Robot 子类

在 `robots.hpp` 中添加新的机器人类：

```cpp
/**
 * @brief 巡检机器人
 * 
 * 设备组成：底盘 + 云台摄像头 + 机械臂
 */
class PatrolRobot : public BaseRobot {
public:
    PatrolRobot(const std::string& robot_id) : BaseRobot(robot_id) {
        // 注册设备处理器
        addHandler("chassis", std::make_shared<ChassisHandler>());
        addHandler("arm", std::make_shared<ArmHandler>("/patrol_arm"));
        addHandler("gimbal", std::make_shared<GimbalHandler>());  // 假设已实现
    }

    std::string getLaunchCommand() override {
        return "ros2 launch patrol_bringup patrol.launch.xml";
    }
};
```

#### 第三步：创建工厂类

在 `robots.hpp` 中添加对应的工厂：

```cpp
class PatrolRobotFactory : public RobotFactory {
public:
    Robot::Ptr createRobot(const std::string& robot_id) override {
        return std::make_shared<PatrolRobot>(robot_id);
    }

    std::vector<std::string> getSupportedTypes() const override {
        return {"patrol"};
    }
};
```

#### 第四步：注册工厂到 BridgeManager

在 `websocket_bridge_node.cpp` 的 `main()` 函数中注册新工厂：

```cpp
int main(int argc, char** argv) {
    // ... 已有代码 ...

    // 创建管理器
    auto manager = std::make_shared<agv_bridge::BridgeManager>();

    // 注册 AGV 机器人工厂
    auto agv_factory = std::make_shared<agv_bridge::AgvRobotFactory>();
    manager->registerFactory("agv", agv_factory);

    // ★ 注册巡检机器人工厂
    auto patrol_factory = std::make_shared<agv_bridge::PatrolRobotFactory>();
    manager->registerFactory("patrol", patrol_factory);

    // ... 其余代码 ...
}
```

#### 第五步：远程注册机器人实例

通过 WebSocket 发送注册消息，让 BridgeManager 创建机器人实例：

```json
{
  "header": {
    "type": "register"
  },
  "payload": {
    "robot_id": "patrol_001",
    "robot_type": "patrol"
  }
}
```

#### 第六步：更新构建配置

在 `CMakeLists.txt` 中确保新依赖已添加，在 `package.xml` 中添加新的依赖声明。

---

## 6. 完整示例：添加带摄像头的复合 AGV

以下是一个完整的端到端示例，展示如何扩展 `AgvRobot` 以支持摄像头控制。

### 6.1 创建 CameraHandler

```cpp
// 在 handlers.hpp 中添加
class CameraHandler : public DeviceHandler {
public:
    CameraHandler(const std::string& topic_prefix = "/camera")
        : topic_prefix_(topic_prefix) {}

    void init(rclcpp::Node* node) override {
        node_ = node;
        // 订阅图像话题（仅记录状态，不转发图像数据）
        image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            topic_prefix_ + "/color/image_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_image_time_ = node_->now();
                image_active_ = true;
            });
    }

    Json::Value handleCommand(const Json::Value& cmd,
                              const agv_protocol::Header& header) override {
        Json::Value result;
        std::string command = cmd["command"].asString();

        if (command == "get_status") {
            std::lock_guard<std::mutex> lock(mutex_);
            result["active"] = image_active_;
            if (image_active_) {
                result["last_image_age_sec"] = 
                    (node_->now() - last_image_time_).seconds();
            }
            result["status"] = "ok";
        } else {
            result["status"] = "error";
            result["message"] = "Unknown command: " + command;
        }
        return result;
    }

    Json::Value getReport() override {
        std::lock_guard<std::mutex> lock(mutex_);
        Json::Value report;
        report["active"] = image_active_;
        return report;
    }

    bool isOnline() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!image_active_) return false;
        // 5 秒内收到过图像认为在线
        return (node_->now() - last_image_time_).seconds() < 5.0;
    }

private:
    std::string topic_prefix_;
    rclcpp::Node* node_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Time last_image_time_;
    bool image_active_ = false;
    mutable std::mutex mutex_;
};
```

### 6.2 注册到 AgvRobot

```cpp
class AgvRobot : public BaseRobot {
public:
    AgvRobot(const std::string& robot_id) : BaseRobot(robot_id) {
        addHandler("chassis", std::make_shared<ChassisHandler>());
        addHandler("lifting", std::make_shared<LiftingHandler>());
        addHandler("camera", std::make_shared<CameraHandler>("/camera"));
    }
    // ...
};
```

### 6.3 测试消息

```json
// 查询摄像头状态
{
  "header": { "robot_id": "agv_001", "type": "command" },
  "payload": {
    "camera": { "command": "get_status" }
  }
}

// 自检：检查摄像头是否在线
{
  "header": { "robot_id": "agv_001", "type": "check" },
  "payload": {
    "camera": { "command": "is_online" }
  }
}
```

---

## 7. 设计原则与注意事项

### 7.1 设计原则

1. **单一职责**：每个 `DeviceHandler` 只负责一种设备类型
2. **开闭原则**：添加新设备/机器人不需要修改已有代码，只需扩展
3. **组合优于继承**：`BaseRobot` 通过组合 handler 实现功能，而非为每种机器人写一个子类
4. **工厂模式**：通过 `RobotFactory` 创建机器人实例，支持运行时动态注册

### 7.2 注意事项

1. **线程安全**：所有 handler 的回调函数可能在不同线程中调用，必须使用 `mutex_` 保护共享数据
2. **ROS2 节点生命周期**：`init()` 中创建的 publisher/subscriber 绑定到传入的 node，确保 node 生命周期覆盖 handler
3. **JSON 库**：使用 `jsoncpp` 库，确保在 `CMakeLists.txt` 中正确链接
4. **消息格式一致性**：`handleCommand()` 的返回值会被直接序列化为 JSON 发送回远程端，确保格式符合协议规范
5. **错误处理**：`handleCommand()` 应返回 `{"status": "error", "message": "..."}` 格式的错误信息

---

## 8. 依赖关系

```
agv_bridge
  ├── agv_protocol    (WebSocket 通信、命令解析)
  ├── rclcpp          (ROS2 客户端库)
  ├── geometry_msgs   (Twist 等消息)
  ├── nav_msgs        (Odometry 消息)
  ├── sensor_msgs     (BatteryState、Image 等消息)
  ├── std_msgs        (基础消息类型)
  ├── Boost           (system、thread)
  └── OpenSSL         (WebSocket TLS 支持)