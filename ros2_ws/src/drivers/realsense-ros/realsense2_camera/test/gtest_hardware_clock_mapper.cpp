#include <gtest/gtest.h>

#include "hardware_clock_mapper.h"

using realsense2_camera::HardwareClockMapper;
using realsense2_camera::DirectEpochTimestampValidator;

TEST(HardwareClockMapper, MapsNormalDeviceClockWithoutChangingEpoch)
{
  HardwareClockMapper mapper;
  const auto first = mapper.map(1000.0, 1700000000000000000LL, 1000000000LL);
  const auto second = mapper.map(1033.25, 1700000000034000000LL, 1034000000LL);
  EXPECT_TRUE(first.valid);
  EXPECT_TRUE(first.epoch_rebuilt);
  EXPECT_EQ(first.epoch, 1U);
  EXPECT_TRUE(second.valid);
  EXPECT_FALSE(second.epoch_rebuilt);
  EXPECT_EQ(second.stamp_ns, 1700000000033250000LL);
  EXPECT_EQ(mapper.rebuild_count(), 0U);
}

TEST(HardwareClockMapper, RebuildsAfterHardwareEpochRegression)
{
  HardwareClockMapper mapper;
  mapper.map(50000.0, 1700000000000000000LL, 1000000000LL);
  const auto before = mapper.map(51000.0, 1700000001000000000LL, 2000000000LL);
  const auto reset = mapper.map(50760.8, 1700000001033400000LL, 2033400000LL);
  const auto after = mapper.map(50794.1, 1700000001066700000LL, 2066700000LL);

  EXPECT_TRUE(before.valid);
  EXPECT_TRUE(reset.valid);
  EXPECT_TRUE(reset.epoch_rebuilt);
  EXPECT_STREQ(reset.reason, "device_clock_regression");
  EXPECT_EQ(reset.epoch, 2U);
  EXPECT_GT(reset.stamp_ns, before.stamp_ns);
  EXPECT_EQ(after.stamp_ns, reset.stamp_ns + 33300000LL);
  EXPECT_EQ(mapper.rebuild_count(), 1U);
}

TEST(HardwareClockMapper, RejectsRebuildWhenHostEpochWentBackward)
{
  HardwareClockMapper mapper;
  mapper.map(1000.0, 1700000000000000000LL, 1000000000LL);
  const auto before = mapper.map(1100.0, 1700000000100000000LL, 1100000000LL);
  const auto invalid = mapper.map(100.0, 1699999999000000000LL, 1200000000LL);
  EXPECT_TRUE(before.valid);
  EXPECT_FALSE(invalid.valid);
  EXPECT_FALSE(invalid.epoch_rebuilt);
  EXPECT_STREQ(invalid.reason, "host_clock_not_monotonic_during_rebuild");
  EXPECT_EQ(mapper.rebuild_count(), 0U);
}

TEST(HardwareClockMapper, RejectsNonFiniteDeviceTime)
{
  HardwareClockMapper mapper;
  const auto invalid = mapper.map(NAN, 1700000000000000000LL, 1000000000LL);
  EXPECT_FALSE(invalid.valid);
  EXPECT_STREQ(invalid.reason, "invalid_input");
}

TEST(HardwareClockMapper, RejectsSmallReorderingWithoutRebuildingEpoch)
{
  HardwareClockMapper mapper;
  const auto first = mapper.map(1000.0, 1700000000000000000LL, 1000000000LL);
  const auto second = mapper.map(1001.0, 1700000000001000000LL, 1001000000LL);
  const auto reordered = mapper.map(1000.5, 1700000000002000000LL, 1002000000LL);

  EXPECT_TRUE(first.valid);
  EXPECT_TRUE(second.valid);
  EXPECT_FALSE(reordered.valid);
  EXPECT_STREQ(reordered.reason, "device_frame_out_of_order");
  EXPECT_EQ(reordered.epoch, 1u);
  EXPECT_EQ(mapper.rebuild_count(), 0u);
}

TEST(DirectEpochTimestampValidator, ReportsRegressionWithoutInventingTimestamp)
{
  DirectEpochTimestampValidator validator;
  EXPECT_TRUE(validator.validate(1700000000000000000LL).valid);
  const auto regression = validator.validate(1699999999760829590LL);
  EXPECT_FALSE(regression.valid);
  EXPECT_STREQ(regression.reason, "direct_epoch_regression");
  EXPECT_EQ(regression.delta_ns, -239170410LL);
  // Invalid input does not poison subsequent valid timestamps.
  EXPECT_TRUE(validator.validate(1700000000033333333LL).valid);
}
