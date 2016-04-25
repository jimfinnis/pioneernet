cmake_minimum_required(VERSION 2.8.3)
project(pioneernet)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs)

include_directories($ENV{HOME}/phd/code/mnist)

## Declare a C++ executable
add_executable(pioneernet_node 
    src/pioneernet_node.cpp
    $ENV{HOME}/phd/code/mnist/backprop.cpp
    $ENV{HOME}/phd/code/mnist/backpropNoBiasHormone.cpp
)

add_dependencies(pioneernet_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pioneernet_node 
  ${catkin_LIBRARIES}
)
