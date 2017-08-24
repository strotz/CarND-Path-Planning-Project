
#include "sensor_fusion.h"
#include "brain.h"
#include "prediction.h"
#include "trajectory.h"
#include "cost_estimator.h"

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

void brain::reset_state(vehicle_state_cref car) {
	last_state_ = car;
	cout << "init done" << endl;
}

unique_ptr<prediction>
brain::generate_prediction(states state, vehicle_state_cref car, sensor_fusion_cref others) {
	switch (state) {
		case KeepLine:
			return generate_keep_in_line(car, others);

		case LineChangeLeft:
			return generate_change_line(car, others, car.lane() - 1);

		case LineChangeRight:
			return generate_change_line(car, others, car.lane() + 1);

		default:
			throw not_implemented();
	}
}

unique_ptr<prediction> brain::run_planning(vehicle car, sensor_fusion_cref others, double start_delay) {

	// TODO: need function
	auto possible_transitions = vector<states>({KeepLine});
	if (last_state_.lane() > 0) {
		possible_transitions.push_back(LineChangeLeft);
	}
	if (last_state_.lane() < 2) {
		possible_transitions.push_back(LineChangeRight);
	}

	int best_cost = 999999999;
	states best_acton;
	unique_ptr<prediction> best_prediction;
	for (auto action : possible_transitions) {

		auto candidate = generate_prediction(action, last_state_, others);
		auto estimator = cost_estimator(action, car, candidate, others, start_delay);
		auto cost = estimator.calculate_cost();
		if (cost < best_cost) {
			best_prediction.reset(candidate.release());
			best_acton = action;
			best_cost = cost;
		}
	}

#ifdef DEV
	cout << "best action: " << best_acton << " with cost " << best_cost << endl;
#endif

	last_state_ = best_prediction->end_state();

#ifdef DEV
	if (best_acton == LineChangeLeft || best_acton == LineChangeRight) {
		cout << endl << "change lane from " << car.lane() << " to " << last_state_.lane() << endl;
		cout << "ego: " << car.s_.value() << endl;
		others.dump();
	}
#endif

	return best_prediction;
}

unique_ptr<prediction>
brain::generate_keep_in_line(const vehicle_state &start_state, sensor_fusion_cref others) {

	auto future = start_state.s_ + start_state.velocity() * preffered_distance;
	auto leader = others.find_nearest_in_range(start_state.lane(), start_state.s_, future);

	timing_profile timing;
	if (leader != nullptr) {
		timing = timing_profile_builder::reach_velocity(start_state.velocity(), leader->velocity(), interval_);
	} else {
		timing = timing_profile_builder::reach_velocity(start_state.velocity(), target_velocity_, interval_);
	}

#ifdef DEV
	cout << "change speed to: " << timing.end_velocity() << " ,duration: "  << timing.total_duration() << endl;
#endif

	std::shared_ptr<trajectory> path = trajectory::maintain_lane(road_, start_state, timing);

#ifdef DEV
	cout << "maintain lane: " << timing.total_distance() << endl;
#endif

	return make_unique<prediction>(start_state, timing, path);
}

unique_ptr<prediction>
brain::generate_change_line(const vehicle_state &start_state, sensor_fusion_cref others, int target_lane) {

	auto future = start_state.s_ + start_state.velocity() * preffered_distance;
	auto leader = others.find_nearest_in_range(target_lane, start_state.s_, future);

	timing_profile timing;
	if (leader != nullptr) {
		timing = timing_profile_builder::reach_velocity(start_state.velocity(), leader->velocity(), interval_);
	} else {
		timing = timing_profile_builder::reach_velocity(start_state.velocity(), target_velocity_, interval_);
	}

#ifdef DEV
	cout << "change speed to: " << timing.end_velocity() << " ,duration: "  << timing.total_duration() << endl;
#endif

	std::shared_ptr<trajectory> path = trajectory::shift_lane(road_, start_state, target_lane, timing);

#ifdef DEV
	cout << "shift lane: " << timing.total_distance() << endl;
#endif

	return make_unique<prediction>(start_state, timing, path);
}


// TODO: check wtf is going on with S on end of round (should it be devided by max?)

bool brain::has_emergencies(const vehicle &car, sensor_fusion_cref others) {
	point start = car.s_;
	point end = last_state_.s_;
	auto leader = others.find_nearest_in_range(car.lane(), start, end);
	if (leader == nullptr) {
		return false;
	}
	auto s = leader->s_ + leader->v_ * max_duration;
	return s < end; // TODO: it is not correct
}

