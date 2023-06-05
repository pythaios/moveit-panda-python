cmake_minimum_required(VERSION 3.1.3)
project(moveit-panda-python)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/panda_pick_and_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
