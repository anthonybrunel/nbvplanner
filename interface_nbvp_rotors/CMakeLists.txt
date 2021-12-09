cmake_minimum_required(VERSION 2.8.3)
project(interface_nbvp_rotors)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  trajectory_msgs
#  mav_msgs
#  tf
#   nbvplanner
#)



include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf
)

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration
  ${catkin_LIBRARIES}
)


 add_dependencies(exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
