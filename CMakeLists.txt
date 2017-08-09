project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

enable_testing()

add_subdirectory(src)
add_subdirectory(test)
