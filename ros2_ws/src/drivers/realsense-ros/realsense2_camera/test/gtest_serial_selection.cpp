#include <gtest/gtest.h>
#include "serial_selection.h"

using realsense2_camera::SerialSelection;

TEST(SerialSelection, BothFieldCamerasRemainEligibleAcrossReplacement)
{
    const SerialSelection selection("_260322276866,352122271190");
    EXPECT_TRUE(selection.accepts("260322276866"));
    EXPECT_TRUE(selection.accepts("352122271190"));
    EXPECT_TRUE(selection.accepts("260322276866"));
    EXPECT_FALSE(selection.accepts("999999999999"));
    EXPECT_FALSE(selection.accepts("26032227686"));
    EXPECT_FALSE(selection.accepts(""));
}

TEST(SerialSelection, SingleSerialStillPinsOneDevice)
{
    const SerialSelection selection("_260322276866");
    EXPECT_TRUE(selection.accepts("260322276866"));
    EXPECT_FALSE(selection.accepts("352122271190"));
}

TEST(SerialSelection, EmptyFilterKeepsAutomaticSelection)
{
    EXPECT_TRUE(SerialSelection().accepts("352122271190"));
}

TEST(SerialSelection, WhitespaceAndDuplicatesDoNotCreateExtraLocks)
{
    const SerialSelection selection(" 260322276866, _352122271190 ,260322276866 ");
    EXPECT_EQ(selection.serials().size(), 2U);
    EXPECT_TRUE(selection.accepts("352122271190"));
}

TEST(SerialSelection, MalformedListsCannotBroadenSelection)
{
    for (const auto value : {",", "260322276866,", ",352122271190", "260322276866,,352122271190", "_", " "})
        EXPECT_THROW(SerialSelection{value}, std::invalid_argument);
}
