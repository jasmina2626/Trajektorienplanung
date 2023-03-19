cmake_minimum_required(VERSION 3.0.2)
project(hochzeit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  unicyclesim
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES hochzeit
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/hochzeit
  include/hochzeit/Trajektorienberechnung
  include/hochzeit/Regler
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable( hochzeit_node src/hochzeit.cpp
                src/Trajektorienberechnung/utility.cpp 
                src/Regler/P2P_controller.cpp
                src/Regler/linDinOut.cpp
                src/init_leader.cpp
               # src/utility_hochzeit.cpp
               )
#add_executable(leader_move_node src/leader_move_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hochzeit_node
  ${catkin_LIBRARIES}
)

##add_executable(follower_regler src/follower_regler.cpp)
##target_link_libraries(follower_regler ${catkin_LIBRARIES})


add_executable(leader_move_node src/leader_move_node.cpp
                                src/Trajektorienberechnung/utility.cpp
                                src/Trajektorienberechnung/ellipse.cpp 
                                src/Trajektorienberechnung/circle.cpp 
                                src/Trajektorienberechnung/line.cpp)
target_link_libraries(leader_move_node ${catkin_LIBRARIES})

