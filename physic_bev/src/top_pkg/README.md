# top_pkg

`top_pkg` 是工作区（FAST-LIVO2 Workspace）的顶层总控 ROS 2 功能包，用于集中管理传感器驱动（Livox MID-360 LiDAR、RealSense D405 Camera）、RViz 可视化及 FAST-LIVO2 建图定位系统的统一启动与参数配置。

---

## 目录结构

```text
top_pkg/
├── CMakeLists.txt         # 编译及安装配置
├── package.xml            # 包依赖定义
├── config/
│   ├── bringup_sensor.yaml # 系统模块使能开关配置
│   └── bringup.rviz        # 默认 RViz 视图配置文件
└── launch/
    ├── bringup_sensor.launch.py # 传感器驱动统一 Launch 引入脚本
    └── bringup.launch.py        # 传感器 + FAST-LIVO2 整体一键启动脚本
```

---

## 参数配置

可以在 `config/bringup_sensor.yaml` 中配置各系统模块的默认使能开关：

```yaml
bringup:
  enable_lidar: true
  enable_camera: false
  enable_rviz: false
```

也可以通过 `ros2 launch` 命令行参数临时覆盖。

---

## 启动方式 Launch

雷达与相机均采用标准 `IncludeLaunchDescription` 方式分别调用官方 Launch 脚本：
- **雷达**：调用 `livox_ros_driver2/launch/msg_MID360_launch.py`
- **相机**：调用 `realsense2_camera/launch/rs_launch.py`

### 1. 单独启动传感器驱动 (Sensors Only)

```bash
# 启动所有已启用的传感器和 RViz
ros2 launch top_pkg bringup_sensor.launch.py

# 命令行覆盖参数（例如关闭 RViz 或单独关闭相机）
ros2 launch top_pkg bringup_sensor.launch.py enable_rviz:=false enable_camera:=false
```

### 2. 整体一键启动 (Sensors + FAST-LIVO2 Mapping)

同时启动传感器驱动、FAST-LIVO2 建图定位节点及 RViz：

```bash
ros2 launch top_pkg bringup.launch.py
```
