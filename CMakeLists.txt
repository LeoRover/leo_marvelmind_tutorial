cmake_minimum_required(VERSION 3.0.2)
project(leo_marvelmind)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS
   robot_localization
#  DEPENDS system_lib
)
