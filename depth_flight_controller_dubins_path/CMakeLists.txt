cmake_minimum_required(VERSION 2.8.3)
project(depth_flight_controller)

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

add_executable(desired_state_publisher src/desired_state_publisher.cpp)
target_link_libraries(desired_state_publisher ${catkin_LIBRARIES})

add_executable(target_finder src/target_finder.cpp)
target_link_libraries(target_finder ${catkin_LIBRARIES})

add_executable(image_clipper src/image_clipper.cpp)
target_link_libraries(image_clipper ${catkin_LIBRARIES})

add_executable(find_max_depth src/find_max_depth.cpp)
target_link_libraries(find_max_depth ${catkin_LIBRARIES})

add_executable(image_prep src/image_prep.cpp)
target_link_libraries(image_prep ${catkin_LIBRARIES})

add_executable(cmd_vel_transformer  src/cmd_vel_transformer.cpp)
target_link_libraries(cmd_vel_transformer ${catkin_LIBRARIES})

cs_add_executable(max_depth_finder src/max_depth_finder.cpp)
target_link_libraries(max_depth_finder ${OpenCV_LIBS})

cs_add_executable(horizon_plotter src/horizon_plotter.cpp)
target_link_libraries(horizon_plotter ${OpenCV_LIBS})

cs_add_executable(c_space_expander src/c_space_expander.cpp)
target_link_libraries(c_space_expander ${OpenCV_LIBS})

include_directories(
        ${catkin_INCLUDE_DIRS}
)
