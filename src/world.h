#ifndef PATH_PLANNING_WORLD_H
#define PATH_PLANNING_WORLD_H

#include <vector>
#include <string>

#include "common.h"
#include "position.h"

using std::vector;
using std::string;

const double max_velocity = 20;

class world {

public:

	world(double max_s = 6945.554) :
		max_s_(max_s),
		map_waypoints_x_(),
		map_waypoints_y_(),
		map_waypoints_s_(),
		map_waypoints_dx_(),
		map_waypoints_dy_()
	{
	}

private:

	// The max s value before wrapping around the track back to 0
	double max_s_; // 6945.554;

public:

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	void load_from_file(const string &map_file_);

private:

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<double> getFrenet(double x, double y, double theta);

	// Transform from Frenet s,d coordinates to Cartesian x,y
	vector<double> getXY(double s, double d) const ;

	int ClosestWaypoint(double x, double y);

	int NextWaypoint(double x, double y, double theta);

public:

	position get_xy_position(double s, double d) const {
		auto xy = getXY(s, d);
		return position(xy[0], xy[1]);
	}

private:

	double distance(double x1, double y1, double x2, double y2);

private:

	// TODO: rely on public API
	FRIEND_TEST(MapWayPoints, StartIndex);
	FRIEND_TEST(MapWayPoints, EndIndex);

private:

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x_;
	vector<double> map_waypoints_y_;
	vector<double> map_waypoints_s_;
	vector<double> map_waypoints_dx_;
	vector<double> map_waypoints_dy_;
};


#endif //PATH_PLANNING_WORLD_H
