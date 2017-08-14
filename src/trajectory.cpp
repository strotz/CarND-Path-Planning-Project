#include "trajectory.h"
#include "position.h"
#include "spline/spline.h"

using namespace std;

trajectory::trajectory(const vehicle_state& ref) :
	ref_state_(ref),
	spline_(tk::spline())
{
}

void trajectory::load_positions(const vector<position>& path) {

	vector<double> x;
	vector<double> y;
	for(auto p : path ) {
		x.push_back(p.x_);
		y.push_back(p.y_);
	}
	spline_.set_points(x, y); // build spline

	auto last = path[path.size()-1];
	auto tan = last.y_ / last.x_;
	distance_coeff_ = 1.0 / sqrt(1 + tan * tan);
}

position trajectory::get_position_at(const double &distance) {

	double x = distance * distance_coeff_;
	double y = spline_(x);
	auto p = position(x, y);
	return p.project_from(ref_state_.p_.x_, ref_state_.p_.y_, ref_state_.orientation_);
}

std::unique_ptr<trajectory> trajectory::maintain_lane(const world& around, const vehicle_state& state, const timing_profile& timing) {
	// make a sparse trajectory in Frenet space
	const int parts = 3;
	const double part = timing.total_distance() / parts;
	const double d = lane_to_d(state.lane());

	vector<position> sparse;

	sparse.push_back(position()); // load current car position

	for (int i = 1; i <= parts; i++) {
		// TODO: d could jump, since car.d and target.d differ
		double s = state.s_ + part * i;
		auto p = around.get_xy_position(s, d); // move to XY
		p = p.project_to(state.p_.x_, state.p_.y_, state.orientation_); // move to CAR
		sparse.push_back(p);
	}

	auto result = std::make_unique<trajectory>(state);
	result->load_positions(sparse);

	auto end = timing.total_distance();
	auto last = result->get_position_at(end);
	auto car_end = result->get_position_at(end - 3.0);

	vehicle_state predicted;
	predicted.p_ = last;
	predicted.orientation_ = atan2(last.y_ - car_end.y_, last.x_ - car_end.x_);
	predicted.s_= state.s_ + end;
	predicted.d_ = d;
	predicted.v_ = timing.end_velocity();

	result->predicted_state_ = predicted;

	// cout << "X from: " << state.p_.x_ << ",to: " << result->predicted_state_.p_.x_ <<

	return result;
}


