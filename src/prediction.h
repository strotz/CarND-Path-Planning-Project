
#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include "vehicle.h"
#include "timing_profile.h"
#include "trajectory.h"

using namespace std;

class prediction {
	vehicle_state start_;
	timing_profile timing_;
	shared_ptr<trajectory> path_;

public:

	vector<position> calculate_trajectory();

	prediction(
		const vehicle_state& start,
		timing_profile& timing,
		shared_ptr<trajectory>& path) :
		start_(start),
		timing_(timing),
		path_(std::move(path))
	{
	}

	double duration() const {
		return timing_.total_duration();
	}

	double total_distance() const {
		return timing_.total_distance();
	}

	const vehicle_state& start_state() const {
		return start_;
	}

	const vehicle_state& end_state() const {
		return path_->predicted();
	}

	const point predict_s_at(const double& delay) const {
		auto distance = timing_.get_distance_at(delay);
		return start_.s_ + distance;
	}
};


#endif //PATH_PLANNING_PREDICTION_H
