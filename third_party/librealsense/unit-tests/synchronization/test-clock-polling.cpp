// Regression: disabling global timestamps must stop firmware clock queries,
// including when more than one sensor shares the device clock fit.
//#cmake: static!
#include "../catch.h"
#include <src/global_timestamp_reader.h>
#include <atomic>
#include <chrono>
#include <thread>

using namespace librealsense;

namespace {
class counting_clock : public global_time_interface
{
public:
    std::atomic<int> queries{0};
    double get_device_time_ms() override { return ++queries; }
};
}

TEST_CASE("disabled_global_clock_does_not_query_firmware", "[global-timestamp]")
{
    counting_clock device;
    time_diff_keeper keeper(&device, 5);
    auto depth = std::make_shared<global_time_option>();
    auto color = std::make_shared<global_time_option>();
    depth->set(0);
    color->set(0);
    keeper.add_enabling_opt(depth);
    keeper.add_enabling_opt(color);
    REQUIRE_FALSE(keeper.is_enabled());
    keeper.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    keeper.stop();
    REQUIRE(device.queries == 0);

    color->set(1);
    REQUIRE(keeper.is_enabled());
    keeper.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (device.queries == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    keeper.stop();
    REQUIRE(device.queries > 0);

    color.reset();
    REQUIRE_FALSE(keeper.is_enabled());
    const auto before = device.queries.load();
    keeper.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    keeper.stop();
    REQUIRE(device.queries == before);
}
