cmake_minimum_required(VERSION 2.8.3)
project(depth_flight_controller_common)

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

add_executable(image_clipper src/image_clipper.cpp)
target_link_libraries(image_clipper ${catkin_LIBRARIES})

add_executable(body_cmd_velocity_publisher src/body_cmd_vel_publisher.cpp)
target_link_libraries(body_cmd_velocity_publisher ${catkin_LIBRARIES})

add_executable(image_prep src/image_prep.cpp)
target_link_libraries(image_prep ${catkin_LIBRARIES})

add_executable(cmd_vel_transformer  src/cmd_vel_transformer.cpp)
target_link_libraries(cmd_vel_transformer ${catkin_LIBRARIES})

cs_add_executable(c_space_expander src/c_space_expander.cpp)
target_link_libraries(c_space_expander ${OpenCV_LIBS})

include_directories(
        ${catkin_INCLUDE_DIRS}
)
