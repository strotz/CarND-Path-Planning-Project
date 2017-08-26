#include "trajectory.h"
#include "position.h"

using namespace std;

trajectory::trajectory(world_cref road, const vehicle_state &ref) :
	road_(road),
	ref_state_(ref),
	spline_(tk::spline())
{
}

void trajectory::load_positions(const vector<position> &path) {

	vector<double> x;
	vector<double> y;
	for (auto p : path) {
		x.push_back(p.x_);
		y.push_back(p.y_);
	}
	spline_.set_points(x, y); // build spline
}

void trajectory::finalize(const point &s, const double &d, const double &distance, const double &end_velocity) {
	auto x = distance;
	auto y = spline_(x);
	auto tan = y / x;
	distance_coeff_ = 1.0 / sqrt(1 + tan * tan);

	auto last = get_position_at(distance);
	auto car_end = get_position_at(distance - car_length);

	vehicle_state predicted;
	predicted.p_ = last;
	predicted.orientation_ = atan2(last.y_ - car_end.y_, last.x_ - car_end.x_);
	predicted.s_ = s + distance;
	predicted.d_ = d;
	predicted.v_ = end_velocity;

	predicted_state_ = predicted;
}

position trajectory::get_position_at(const double &distance) {

	double x = distance * distance_coeff_;
	double y = spline_(x);
	auto p = position(x, y);
	return p.project_from(ref_state_.p_.x_, ref_state_.p_.y_, ref_state_.orientation_);
}

std::unique_ptr<trajectory>
trajectory::maintain_lane(world_cref road, const vehicle_state &car, const timing_profile &timing) {
	// make a sparse trajectory in Frenet space
	const int parts = 3;
	const double d = lane_to_d(car.lane());

	vector<position> sparse;

	sparse.push_back(position(-car_length, 0)); // position of the car back in CAR coordinates
	sparse.push_back(position(0, 0)); // position in CAR coordinates

	for (int i = 1; i <= parts; i++) {
		point s = car.s_ + look_ahead_distance * i;
		auto p = road.get_xy_position(s, d); // move to XY
		p = p.project_to(car.p_.x_, car.p_.y_, car.orientation_); // move to CAR coordinates
		sparse.push_back(p);
	}

	auto result = make_unique<trajectory>(road, car);
	result->load_positions(sparse);
	result->finalize(car.s_, d, timing.total_distance(), timing.end_velocity());
	return result;
}

std::unique_ptr<trajectory>
trajectory::shift_lane(world_cref road, const vehicle_state &car, int target_lane, const timing_profile &timing) {
	// make a sparse trajectory in Frenet space
	const int parts = 3;
	const double d = lane_to_d(target_lane);

	vector<position> sparse;

	sparse.push_back(position(-car_length, 0)); // position of the car back in CAR coordinates
	sparse.push_back(position()); // position in CAR coordinates

	for (int i = 1; i <= parts; i++) {
		point s = car.s_ + look_ahead_distance * i;
		auto p = road.get_xy_position(s, d); // move to XY
		p = p.project_to(car.p_.x_, car.p_.y_, car.orientation_); // move to CAR coordinates
		sparse.push_back(p);
	}

	auto result = make_unique<trajectory>(road, car);
	result->load_positions(sparse);
	result->finalize(car.s_, d, timing.total_distance(), timing.end_velocity());
	return result;
}

frenet trajectory::get_frenet_at(const double &distance) {
	auto xy = get_position_at(distance);
	auto car_end_xy = get_position_at(distance - car_length);
	double theta = atan2(xy.y_ - car_end_xy.y_, xy.x_ - car_end_xy.x_);
	return road_.get_frenet_position(xy, theta);
}
