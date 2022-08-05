cmake_minimum_required(VERSION 3.10)
project(udacity_cpp_ekf)

include_directories(
  include
  /usr/include/eigen3/
)

add_executable(kalman_filter_1d src/kalman_filter_1d.cpp)

add_executable(kalman_filter
  src/kalman_filter_main.cpp
  src/kalman_filter.cpp
  src/tracking.cpp
)

add_executable(ekf
  src/ekf_main.cpp
  src/fusion_ekf.cpp
  src/kalman_filter.cpp
  src/tools.cpp
)

target_link_libraries(ekf uWS z)
