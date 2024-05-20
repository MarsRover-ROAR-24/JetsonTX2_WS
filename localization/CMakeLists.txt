cmake_minimum_required(VERSION 3.0)
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
  tf2
  tf2_ros
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
  tf2
  tf2_ros
  INCLUDE_DIRS include 
)



include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add_executable(ROAR_main src/ROAR_main.cpp src/ROAR_UKF.cpp src/Quaternion.cpp)
# add_dependencies(ROAR_main localization_generate_messages_cpp)
# target_link_libraries(ROAR_main ${catkin_LIBRARIES} Eigen3::Eigen)
# target_include_directories(ROAR_main PRIVATE include)

# add_executable(ROAR_main_simulation src/ROAR_main_simulation.cpp src/ROAR_UKF.cpp src/Quaternion.cpp)
# target_link_libraries(ROAR_main_simulation ${catkin_LIBRARIES} Eigen3::Eigen)
# target_include_directories(ROAR_main_simulation PRIVATE include)

# add_executable(OOSM src/OOSM.cpp)
# add_dependencies(OOSM localization_generate_messages_cpp)
# target_link_libraries(OOSM ${catkin_LIBRARIES})
# target_include_directories(OOSM PRIVATE include)

# add_executable(ROAR_UKF src/ROAR_main src/ROAR_UKF.cpp src/Quaternion.cpp)
# target_link_libraries(ROAR_UKF ${catkin_LIBRARIES} Eigen3::Eigen)
# target_include_directories(ROAR_UKF PRIVATE include)


# catkin_install_python(PROGRAMS
#   src/script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


