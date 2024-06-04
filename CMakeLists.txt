cmake_minimum_required(VERSION 3.0.2)
project(hrpsys_ext_rtc)

# Compile as C++11, supported in ROS Kinetic and newer
 set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp)

# initialize rtmbuild2 (generate_messages is called)
rtmbuild2_init()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES whole_body_master_slave_choreonoid
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

###########
## Build ##
###########

add_subdirectory(rtc)
