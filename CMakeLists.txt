cmake_minimum_required (VERSION 2.8.11)
project (satellite_manager)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin COMPONENTS
    roscpp
    rosbag
    inertial_sense
    std_msgs
    geometry_msgs
)

include_directories(include
    ${GTEST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    lib/RTKLIB/src
    )


add_library(satellite_manager
    src/satellite_manager.cpp
    lib/RTKLIB/src/ephemeris.c
    lib/RTKLIB/src/rtkcmn.c
    lib/RTKLIB/src/preceph.c
    lib/RTKLIB/src/qzslex.c
    lib/RTKLIB/src/rtcm.c
    lib/RTKLIB/src/rtcm2.c
    lib/RTKLIB/src/rtcm3.c
    lib/RTKLIB/src/rtcm3e.c
    lib/RTKLIB/src/sbas.c
    )

add_executable(satellite_manager_test test/test.cpp)
target_link_libraries(satellite_manager_test
    ${GTEST_LIBRARIES}
    gtest
    gtest_main
    pthread
)


if (${catkin_FOUND})
    include_directories(${catkin_INCLUDE_DIRS})

    add_library(satellite_manager_ros src/satellite_manager_ros.cpp)
    target_link_libraries(satellite_manager_ros
        ${catkin_LIBRARIES}
        satellite_manager
        pthread)

    add_executable(satellite_manager_node src/satellite_manager_node.cpp)
    target_link_libraries(satellite_manager_node
        ${catkin_LIBRARIES}
        satellite_manager_ros
        pthread)

    add_executable(satellite_manager_rosbag src/satellite_manager_rosbag.cpp)
    target_link_libraries(satellite_manager_rosbag
        ${catkin_LIBRARIES}
        satellite_manager_ros
        pthread)
endif()


