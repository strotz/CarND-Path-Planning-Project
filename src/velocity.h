#ifndef PATH_PLANNING_VELOCITY_H_H
#define PATH_PLANNING_VELOCITY_H_H

class velocity {
public:

	velocity(const double& v) : v_(v) {
	}

	bool same_as(const velocity& other) const {
		return same(*this, other);
	}

	operator double&() { return v_; }
	// operator const double&() const { return v_; }
	operator double() const { return v_; }

private:

	double v_;

public:

	static bool same(const velocity& one, const velocity& two) {
		return fabs(one.v_ - two.v_) < 0.2;
	}
};


#endif //PATH_PLANNING_VELOCITY_H_H
