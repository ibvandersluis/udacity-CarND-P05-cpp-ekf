cmake_minimum_required(VERSION 3.10)
project(udacity_cpp_ekf)

include_directories(
  /usr/local/include/eigen3
)

add_executable(kalman_filter src/kalman_filter.cpp)
