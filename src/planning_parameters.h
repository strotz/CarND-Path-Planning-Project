
#ifndef PATH_PLANNING_PLANNING_PARAMETERS_H
#define PATH_PLANNING_PLANNING_PARAMETERS_H

const double speed_limit = 47; // MPH

const double max_acceleration = 7.0;
const double min_acceleration = -7.0;

const double max_duration = 0.5;
const double step_duration = 0.02;
const double required_steps = max_duration / step_duration;

const double preffered_distance = max_duration * 2;

const double lane_width = 4.0;
const int lanes = 3;


#endif //PATH_PLANNING_PLANNING_PARAMETERS_H
