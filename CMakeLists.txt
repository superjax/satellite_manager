cmake_minimum_required (VERSION 2.8.11)
project (satellite_manager)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include
    ${GTEST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    lib/RTKLIB/src
    )

add_library(rtklib
    lib/RTKLIB/src/ephemeris.c
    lib/RTKLIB/src/rtkcmn.c
    )

add_library(satellite_manager
    src/satellite_manager.cpp
    )
add_executable(satellite_manager_test test/test.cpp)
target_link_libraries(satellite_manager_test
    ${GTEST_LIBRARIES}
    gtest
    gtest_main
    pthread
    )
