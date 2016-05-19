cmake_minimum_required(VERSION 2.8.3)
project(pioneernet)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs
    std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pioneernet
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
    $ENV{HOME}/phd/code/mnist
    $ENV{HOME}/phd/code/pioneernet
)

## Declare a C++ executable
add_executable(pioneernet_node 
    src/pioneernet_node.cpp
    $ENV{HOME}/phd/code/mnist/backprop.cpp
    $ENV{HOME}/phd/code/mnist/plainbp.cpp
    $ENV{HOME}/phd/code/mnist/backpropNoBiasHormone.cpp
    $ENV{HOME}/phd/code/mnist/loadsave.cpp
)

add_dependencies(pioneernet_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pioneernet_node 
  ${catkin_LIBRARIES}
)

add_executable(bridge_node
    src/bridge_node.cpp
)

add_dependencies(bridge_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bridge_node 
  ${catkin_LIBRARIES}
)
