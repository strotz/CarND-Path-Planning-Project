#include "common.h"

static bool verbose_ = false;


std::ostream& out() {
	static null_stream null_;

	if (verbose_)
		return std::cout;

	return  null_;
}

void enable_verbose() {
	verbose_ = true;
}

void disable_verbose() {
	verbose_ = false;
}
