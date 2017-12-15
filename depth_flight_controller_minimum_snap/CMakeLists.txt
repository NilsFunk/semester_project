cmake_minimum_required(VERSION 2.8.3)
project(depth_flight_controller_minimum_snap)

find_package(catkin_simple REQUIRED COMPONENTS
        rospy
        roscpp
        genmsg
        geometry_msgs
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
)

catkin_package()
catkin_simple()

cs_install()
cs_export()

add_executable(snap_trajectory_planner src/snap_trajectory_planner.cpp)
target_link_libraries(snap_trajectory_planner ${catkin_LIBRARIES})

add_executable(target_finder src/target_finder.cpp)
target_link_libraries(target_finder ${catkin_LIBRARIES})

include_directories(
        ${catkin_INCLUDE_DIRS}
)
