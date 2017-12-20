cmake_minimum_required(VERSION 2.8.3)
project(depth_flight_controller_dubins_path)

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

add_executable(desired_state_publisher_yaw_adjustment src/desired_state_publisher_yaw_adjustment.cpp)
target_link_libraries(desired_state_publisher_yaw_adjustment ${catkin_LIBRARIES})

add_executable(desired_state_publisher src/desired_state_publisher.cpp)
target_link_libraries(desired_state_publisher ${catkin_LIBRARIES})

add_executable(target_finder src/target_finder.cpp)
target_link_libraries(target_finder ${catkin_LIBRARIES})

include_directories(
        ${catkin_INCLUDE_DIRS}
)
