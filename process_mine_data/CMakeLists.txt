cmake_minimum_required(VERSION 3.0.2)
project(process_mine_data)


set(CMAKE_CXX_STANDARD 14)

# LAZ library
add_subdirectory(3rd/LASzip)
include_directories(3rd/LASzip/include)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES process_mine_dataroscpp
#  CATKIN_DEPENDS pcl_ros sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable(release_data_laz src/release_data_laz.cpp src/SaveLAZ.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(release_data_laz ${catkin_LIBRARIES} laszip  ${Eigen_LIBRARIES})

