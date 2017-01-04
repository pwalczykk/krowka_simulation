cmake_minimum_required(VERSION 2.8.3)
project(krowka_model)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
    roscpp
    dynamic_reconfigure
    cv_bridge
    image_transport

)

find_package(OpenCV 3.0.0 REQUIRED)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
    src/krowka_slider_controller/DRC_Krowka.cfg
)

catkin_package()

add_executable(krowka_slider_controller
    src/krowka_slider_controller/Main.cpp)
target_link_libraries(krowka_slider_controller ${catkin_LIBRARIES})
add_dependencies(krowka_slider_controller ${krowka_model}_gencfg)

add_executable(krowka_inverse_kinematics
    src/krowka_inverse_kinematics/Main.cpp)
target_link_libraries(krowka_inverse_kinematics ${catkin_LIBRARIES})

#
# add_executable(display_image_opencv
#     src/display_image_opencv/Main.cpp)
# target_link_libraries(display_image_opencv ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
