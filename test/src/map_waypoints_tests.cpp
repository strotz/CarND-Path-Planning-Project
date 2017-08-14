#include "gtest/gtest.h"

#include <world.h>

TEST(MapWayPoints, StartIndex) {
	world z;
	z.map_waypoints_s_.push_back(0.0);
	z.map_waypoints_s_.push_back(1.0);
}

TEST(MapWayPoints, EndIndex) {
	world z;
	z.map_waypoints_s_.push_back(0.0);
	z.map_waypoints_s_.push_back(1.0);
}
