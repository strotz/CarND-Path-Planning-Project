#ifndef PATH_PLANNING_COMMON_H
#define PATH_PLANNING_COMMON_H

#define FRIEND_TEST(test_case_name, test_name) friend class test_case_name##_##test_name##_Test

#include <stdexcept>
#include <memory>
#include <iostream>
#include <iomanip>

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


class null_buffer : public std::streambuf
{
public:
	int overflow(int c) { return c; }
};

class null_stream : public std::ostream
{
public:
	null_stream() : std::ostream(&m_sb)
	{
	}

	null_stream(const null_stream& other) : std::ostream(&m_sb)
	{
	}

private:
	null_buffer m_sb;
};

void enable_verbose();
void disable_verbose();
std::ostream& out();

// #define DEV

#endif //PATH_PLANNING_COMMON_H
