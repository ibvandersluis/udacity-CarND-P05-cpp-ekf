cmake_minimum_required(VERSION 3.10)
project(udacity_cpp_ekf)

include_directories(
  /usr/include/eigen3/
)

add_executable(kalman_filter_1d src/kalman_filter_1d.cpp)
