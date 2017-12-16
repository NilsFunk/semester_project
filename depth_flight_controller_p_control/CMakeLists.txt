cmake_minimum_required(VERSION 2.8.3)
project(depth_flight_controller_p_control)

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

cs_add_executable(max_depth_finder src/max_depth_finder.cpp)
target_link_libraries(max_depth_finder ${OpenCV_LIBS})

cs_add_executable(horizon_plotter src/horizon_plotter.cpp)
target_link_libraries(horizon_plotter ${OpenCV_LIBS})

include_directories(
        ${catkin_INCLUDE_DIRS}
)
