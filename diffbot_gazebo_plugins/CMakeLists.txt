cmake_minimum_required(VERSION 3.0.2)
project(diffbot_gazebo_plugins)

# Gazebo 11's headers need C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# gazebo_dev finds Gazebo and adds its include folders and libraries to the catkin_* variables
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  sensor_msgs
)

# Nothing links against the plugin library: Gazebo loads it at run time
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# libdiffbot_gazebo_ros_imu_sensor.so, the simulated IMU's plugin in DiffBot's and Remo's models.
# Gazebo looks for a plugin's file name in GAZEBO_PLUGIN_PATH and then on LD_LIBRARY_PATH. The
# workspace's setup.bash puts the devel and the install space's lib folder on LD_LIBRARY_PATH.
add_library(diffbot_gazebo_ros_imu_sensor src/gazebo_ros_imu_sensor.cpp)
target_link_libraries(diffbot_gazebo_ros_imu_sensor ${catkin_LIBRARIES})

install(TARGETS diffbot_gazebo_ros_imu_sensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# The Apache License 2.0 asks to pass its text on with the plugin
install(FILES LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
