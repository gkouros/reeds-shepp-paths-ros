cmake_minimum_required(VERSION 2.8.3)
project(reeds_shepp_paths_ros)

# C++ 11 required for OMPL
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
)

find_package(OMPL)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
  # DEPENDS
  LIBRARIES
    reeds_shepp_paths_ros
  )

add_library(reeds_shepp_paths_ros src/reeds_shepp_paths_ros.cpp)
target_link_libraries(reeds_shepp_paths_ros ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo reeds_shepp_paths_ros ${catkin_LIBARRIES})
