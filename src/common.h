#ifndef PATH_PLANNING_COMMON_H
#define PATH_PLANNING_COMMON_H

#define FRIEND_TEST(test_case_name, test_name) friend class test_case_name##_##test_name##_Test

class not_implemented : public std::runtime_error
{
public:

	not_implemented() : std::runtime_error("not implemented") {
	}
};

#endif //PATH_PLANNING_COMMON_H
