cmake_minimum_required(VERSION 2.8.3)
project(reeds_shepp_paths_ros)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    costmap_2d
    base_local_planner
    geometry_msgs
    nav_msgs
)

find_package(OMPL)
if (NOT OMPL_FOUND)
  message(SEND_ERROR "Failed to find OMPL! Install OMPL and try again!")
endif ()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    costmap_2d
    base_local_planner
    geometry_msgs
    nav_msgs
  LIBRARIES
    ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${PROJECT_NAME} ${catkin_LIBARRIES})
