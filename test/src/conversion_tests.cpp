#include "gtest/gtest.h"

#include <conversion.h>

TEST(Conversion, DToLane) {
	EXPECT_EQ(d_to_lane(2), 0);
}

TEST(Conversion, LaneToD) {
	EXPECT_EQ(lane_to_d(0), 3.0);
}