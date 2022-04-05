cmake_minimum_required(VERSION 3.0.2)
project(real_mcts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pcutils src/pcutils.cpp)
add_library(mcts src/mcts.cpp)

add_executable(main src/main.cpp)
target_link_libraries(main pcutils mcts ${catkin_LIBRARIES})
