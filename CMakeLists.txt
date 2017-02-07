cmake_minimum_required(VERSION 2.8.3)
project(pioneernet)

include(FindPkgConfig)

pkg_search_module(SDL REQUIRED sdl2)
pkg_search_module(SDL2_TTF REQUIRED SDL2_ttf)
pkg_search_module(SDL2_GFX REQUIRED SDL2_gfx)

set(SDL_CF ${SDL_CFLAGS} ${SDL2_TTF_CFLAGS} ${SDL2_GFX_CFLAGS})
set(SDL_LD ${SDL_LDFLAGS} ${SDL2_TTF_LDFLAGS} ${SDL2_GFX_LDFLAGS})

add_definitions("-std=c++11")
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
    $ENV{HOME}/phd/code/mnist/backpropNoBiasHormoneHard.cpp
    $ENV{HOME}/phd/code/mnist/backpropNoBiasHormoneSigCentred.cpp
    $ENV{HOME}/phd/code/mnist/loadsave.cpp
)

add_dependencies(pioneernet_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pioneernet_node 
  angort dl m diamondapparatus
  ${catkin_LIBRARIES}
)

add_executable(bridge_node
    src/bridge_node.cpp
)

add_dependencies(bridge_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bridge_node 
  diamondapparatus 
  ${catkin_LIBRARIES}
)

add_executable(showsonar_node
    src/showsonar_node.cpp
    ~/phd/code/wheelyPioneer/sdl.cpp
)
target_compile_options(showsonar_node PUBLIC ${SDL_CF})

include_directories("~/phd/code/wheelyPioneer")
add_dependencies(showsonar_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(showsonar_node 
  ${catkin_LIBRARIES} ${SDL_LD}
)
