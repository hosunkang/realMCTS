cmake_minimum_required(VERSION 3.0.2)
project(real_mcts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES real_mcts
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
