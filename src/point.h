
#ifndef PATH_PLANNING_DISTANCE_H
#define PATH_PLANNING_DISTANCE_H

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

class point {

	double s_;

public:

	point() : s_(0.0) {}

	point(const double& s) : s_(s)
	{
	}

	point(const point& other) : s_(other.s_)
	{
	}

	const double& value() const {
		return s_;
	}

	point operator+(const point& other) const
	{
		return point(s_ + other.s_);
	}

	point operator-(const point& other) const
	{
		return point(s_ - other.s_);
	}

	point operator+(const double& other) const
	{
		return point(s_ + other);
	}

	point operator-(const double& other) const
	{
		return point(s_ - other);
	}

	bool operator<(const point& other) const {
		return s_ < other.s_;
	}

	bool operator>(const point& other) const {
		return s_ > other.s_;
	}
};

#endif //PATH_PLANNING_DISTANCE_H
