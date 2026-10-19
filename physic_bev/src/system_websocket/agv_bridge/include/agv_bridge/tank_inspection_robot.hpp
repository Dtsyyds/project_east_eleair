#pragma once
#include "agv_bridge/robot.hpp"
#include <memory>
namespace agv_bridge {
// Passive device attachment: no sensor owner, mapping or collection is started.
class TankInspectionRobot final : public Robot {
public:
    TankInspectionRobot();
    ~TankInspectionRobot() override;
    void init(rclcpp::Node* node) override;
    std::string getLaunchCommand() override { return ""; }
    Json::Value handleCommand(const agv_protocol::RawMessage&) override;
    Json::Value handleCheck(const agv_protocol::RawMessage&) override;
    std::vector<std::string> generateReports(agv_protocol::CommandParser&) override;
    std::vector<std::string> generateEvents(agv_protocol::CommandParser&) override { return {}; }
    std::string getRobotId() const override { return "tank_inspection_robot"; }
    void tick() override;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
