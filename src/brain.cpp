
#include "brain.h"
#include "trajectory.h"

vector<position> prediction::calculate_trajectory() {
	auto result = vector<position>();

	int steps = timing_.total_duration() / step_duration;
	for (int i = 1; i <= steps; i++) {
		auto delay = i * step_duration;
		auto distance = timing_.get_distance_at(delay);
		auto next_position = path_->get_position_at(distance);
		result.push_back(next_position);
	}
	return result;
}

void brain::reset_state(vehicle_state_cref car)
{
	last_state_ = car;
	cout << "init done" << endl;
}

unique_ptr<prediction> brain::generate_prediction(states state, vehicle_state_cref car, const vector<detected_vehicle> &others) {
	switch (state) {
		case KeepLine:
			return generate_keep_in_line(car, others);

		default:
			throw not_implemented();
	}
}

unique_ptr<prediction> brain::run_planning(vehicle_state_cref car, const vector<detected_vehicle>& others) {

	// TODO: need function
	auto possible_transitions = vector<states>({KeepLine});
	double best_cost = 99999;
	unique_ptr<prediction> best_prediction;
	for (auto state : possible_transitions) {

		auto candidate = generate_prediction(state, last_state_, others);
		auto cost = 0; // TODO: for now
		if (cost < best_cost) {
			best_prediction.reset(candidate.release());
		}
	}
	last_state_ = best_prediction->end_state();
	return best_prediction;
}

unique_ptr<prediction> brain::generate_keep_in_line(const vehicle_state &car, const vector<detected_vehicle>& others) {

	bool in_front = false;
	detected_vehicle leader;
	for(auto c : others) {
		if (car.lane() == c.lane() && car.s_ < c.s_) {
			if (!in_front) {
				leader = c;
				in_front = true;
			} else {
				if (c.s_ < leader.s_) {
					leader = c;
				}
			}
		}
	}

	timing_profile timing;
	if (in_front && timing_profile_builder::too_close(car.s_, car.velocity(), leader.s_)) {
		timing = timing_profile_builder::reach_velocity(car.velocity(), leader.velocity(), interval_);
	} else {
		timing = timing_profile_builder::reach_velocity(car.velocity(), target_velocity_, interval_);
	}
	cout << "change speed to: " << timing.end_velocity() << " ,duration: "  << timing.total_duration() << endl;

	std::shared_ptr<trajectory> path = trajectory::maintain_lane(road_, car, timing);
	cout << "maintain lane: " << timing.total_distance() << endl;

	return make_unique<prediction>(timing, path);
}

//	if (predicted.lane() == target.lane()) {
//	} else {
//		path = trajectory::shift_lane(road, predicted, target.lane(), *timing);
//		cout << "shift lane: " << timing->total_distance() << endl;
//	}

