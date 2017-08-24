set(test_sources
        main.cpp
        map_waypoints_tests collision.cpp)

add_executable(path_planning_tests ${test_sources})

include_directories(../../src)

target_link_libraries(path_planning_tests
        libgtest
        libgmock
        )

add_test(NAME path_planning_tests
        COMMAND path_planning_tests)