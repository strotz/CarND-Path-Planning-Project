#include <math.h>

#ifndef PATH_PLANNING_CONVERSION_H
#define PATH_PLANNING_CONVERSION_H

inline double mph_to_ms(const double& speed) {
	return speed * 0.44704;
}

const double lane_width = 4.0;

inline int d_to_lane(const double& d) {
	return d / lane_width;
}

inline double lane_to_d(int lane) {
	return lane_width /2 + lane * lane_width;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

inline double rad2deg(double x) { return x * 180 / pi(); }


#endif //PATH_PLANNING_CONVERSION_H
