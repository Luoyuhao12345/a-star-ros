cmake_minimum_required(VERSION 3.0.2)
project(a_star)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS  roscpp 
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_planner
  src/main.cpp
  src/planner.cpp
)
# add_dependencies(astar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(astar_planner
  ${catkin_LIBRARIES}
)
