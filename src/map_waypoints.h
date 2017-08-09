#ifndef PATH_PLANNING_MAP_VIEWPOINTS_H
#define PATH_PLANNING_MAP_VIEWPOINTS_H

#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class map_viewpoints
{
public:
	map_viewpoints(double max_s = 6945.554) :
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
	void load_from_file(const string& map_file_) {

		ifstream in_map_(map_file_.c_str(), ifstream::in);

		string line;
		while (getline(in_map_, line)) {
			istringstream iss(line);
			double x;
			double y;
			float s;
			float d_x;
			float d_y;
			iss >> x;
			iss >> y;
			iss >> s;
			iss >> d_x;
			iss >> d_y;
			map_waypoints_x_.push_back(x);
			map_waypoints_y_.push_back(y);
			map_waypoints_s_.push_back(s);
			map_waypoints_dx_.push_back(d_x);
			map_waypoints_dy_.push_back(d_y);
		}
	}

	int find_current_start(double s) {
		double s_norm = fmod(s, max_s_);
		for(int i = 0; i < map_waypoints_s_.size()-1; ++i) {
			if (map_waypoints_s_[i + 1] > s_norm) {
				return i;
			}
		}
		return map_waypoints_s_.size()-1;
	}

	int find_current_end(double s) {
		int start = find_current_start(s);
		if (start == map_waypoints_s_.size()-1) {
			return 0;
		} else {
			return start + 1;
		}
	}

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x_;
	vector<double> map_waypoints_y_;
	vector<double> map_waypoints_s_;
	vector<double> map_waypoints_dx_;
	vector<double> map_waypoints_dy_;
};

#endif //PATH_PLANNING_MAP_VIEWPOINTS_H
