# East Eleair

罐体巡检工程源码，包含 ROS 2 传感器集成、视觉检测、D405 视觉与深度融合及 WebSocket 上位机接口。

## 工程入口

- [工程功能包说明](PROJECT_FUNCTION_PACKAGE_GUIDE_CN.md)
- [操作手册](操作手册.md)
- [整机采集与 WebSocket 操作说明](整机采集与WebSocket操作说明.md)
- [巡检总控与部署](tank_inspection_system/README.md)
- [视觉巡检](tank_vision_inspection/README.md)
- [D405 融合实验框架](d405_seam_tracking/README.md)
- 传感器与 ROS 包：`physic_bev/src/`
- 第三方依赖源码：`third_party/`，许可证保留于对应目录。

## 仓库存储范围

本仓库保存当前工作目录中的源码、配置、测试和文档快照。原有子仓库的 Git 元数据及历史保留在本机，顶层仓库直接保存其源码文件。

虚拟环境、构建和安装目录、运行日志、采集数据、大型报告、离线安装包和厂商 SDK 二进制不上传，忽略规则见 `.gitignore`；这些文件仍保留在本机。离线依赖清单和现有验证报告可能引用这些本地材料，克隆仓库不等于恢复可直接运行的板端环境。

部署前请按各模块文档准备 ROS 2、厂商 SDK 和 Python 依赖，并根据实际设备配置本地路径。历史文档中的测试结果仅代表对应记录的环境与时间。
