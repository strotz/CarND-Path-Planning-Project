//
// Created by alstrots on 8/11/17.
//

#ifndef PATH_PLANNING_WORLD_H
#define PATH_PLANNING_WORLD_H

#include <vector>
#include <string>
#include <math.h>

using std::vector;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

class world {

public:

	static double deg2rad(double x) { return x * pi() / 180; }

	double rad2deg(double x) { return x * 180 / pi(); }

	world(double max_s = 6945.554) :
		max_s_(max_s),
		map_waypoints_x_(),
		map_waypoints_y_(),
		map_waypoints_s_(),
		map_waypoints_dx_(),
		map_waypoints_dy_()
	{
	}

	// The max s value before wrapping around the track back to 0
	double max_s_; // 6945.554;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	void load_from_file(const string &map_file_);

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<double> getFrenet(double x, double y, double theta);

	// Transform from Frenet s,d coordinates to Cartesian x,y
	vector<double> getXY(double s, double d);



	int find_current_start(double s);

	int find_current_end(double s);

	int ClosestWaypoint(double x, double y);

	int NextWaypoint(double x, double y, double theta);



private:

	double distance(double x1, double y1, double x2, double y2);

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x_;
	vector<double> map_waypoints_y_;
	vector<double> map_waypoints_s_;
	vector<double> map_waypoints_dx_;
	vector<double> map_waypoints_dy_;
};


#endif //PATH_PLANNING_WORLD_H
