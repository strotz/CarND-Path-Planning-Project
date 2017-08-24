#ifndef PATH_PLANNING_COMMON_H
#define PATH_PLANNING_COMMON_H

#define FRIEND_TEST(test_case_name, test_name) friend class test_case_name##_##test_name##_Test

#include <stdexcept>
#include <memory>

class not_implemented : public std::runtime_error
{
public:

	not_implemented() : std::runtime_error("not implemented") {
	}
};

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// #define DEV

#endif //PATH_PLANNING_COMMON_H
