#include "gtest/gtest.h"

#include <map_waypoints.h>

TEST(MapWayPoints, StartIndex) {
	map_viewpoints z;
	z.map_waypoints_s_.push_back(0.0);
	z.map_waypoints_s_.push_back(1.0);

	EXPECT_EQ(z.find_current_start(0.0), 0);
	EXPECT_EQ(z.find_current_start(0.5), 0);
	EXPECT_EQ(z.find_current_start(1.0), 1);
	EXPECT_EQ(z.find_current_start(1.5), 1);
	EXPECT_EQ(z.find_current_start(z.max_s_ + 0.5), 0);
}

TEST(MapWayPoints, EndIndex) {
	map_viewpoints z;
	z.map_waypoints_s_.push_back(0.0);
	z.map_waypoints_s_.push_back(1.0);

	EXPECT_EQ(z.find_current_end(0.0), 1);
	EXPECT_EQ(z.find_current_end(0.5), 1);
	EXPECT_EQ(z.find_current_end(1.0), 0);
	EXPECT_EQ(z.find_current_end(1.5), 0);
	EXPECT_EQ(z.find_current_end(z.max_s_ + 0.5), 1);
}
