set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
        common.h
        conversion.h
        cost_estimator.cpp
        cost_estimator.h
        prediction.h
        position.h
        point.h
        main.cpp
        states.h
        timing_profile.cpp
        timing_profile.h
        world.cpp
        world.h
        vehicle.cpp
        vehicle.h
        trajectory.cpp
        trajectory.h
        velocity.h
        brain.cpp
        brain.h
        sensor_fusion.cpp sensor_fusion.h common.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)

install(TARGETS path_planning DESTINATION bin)