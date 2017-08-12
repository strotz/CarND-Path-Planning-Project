//
// Created by alstrots on 8/11/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_BUILDER_H
#define PATH_PLANNING_TRAJECTORY_BUILDER_H

#include <vector>
#include "vehicle.h"

using std::vector;

class trajectory {
public:

	position get_position_at(const double& delay);
};

class trajectory_builder {
public:

	static trajectory build_trajectory(vector<position> sparse_points);
};


#endif //PATH_PLANNING_TRAJECTORY_BUILDER_H
