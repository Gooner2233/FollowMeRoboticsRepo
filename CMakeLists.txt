cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(follow_me)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(mbot_bridge REQUIRED)

# 1D follow me executable.
add_executable(follow_1D src/follow_1D.cpp
  src/common/utils.cpp
)
target_link_libraries(follow_1D
  mbot_bridge_cpp
)
target_include_directories(follow_1D PRIVATE
  include
)

# Drive star executable.
add_executable(drive_star
  src/drive_star.cpp
)
target_link_libraries(drive_star
  mbot_bridge_cpp
)
target_include_directories(drive_star PRIVATE
  include
)

# 2D follow me executable.
add_executable(follow_2D src/follow_2D.cpp
  src/common/utils.cpp
)
target_link_libraries(follow_2D
  mbot_bridge_cpp
)
target_include_directories(follow_2D PRIVATE
  include
)

if (CMAKE_BUILD_TYPE MATCHES Test)
endif()
