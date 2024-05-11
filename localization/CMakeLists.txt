cmake_minimum_required(VERSION 3.0.2)
project(localization
	LANGUAGES CXX)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roar_msgs
  geometry_msgs
  message_generation
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  roar_msgs
  geometry_msgs
  message_runtime
  INCLUDE_DIRS include 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/ROAR_UKF.cpp src/Quaternion.cpp)
add_dependencies(main roar_msgs_generate_messages_cpp)
target_link_libraries(main ${catkin_LIBRARIES} Eigen3::Eigen)
target_include_directories(main PRIVATE include)

add_executable(ROAR_UKF src/main src/ROAR_UKF.cpp src/Quaternion.cpp)
target_link_libraries(ROAR_UKF ${catkin_LIBRARIES} Eigen3::Eigen)
target_include_directories(ROAR_UKF PRIVATE include)

# catkin_install_python(PROGRAMS
#   src/script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
