cmake_minimum_required(VERSION 3.8)
project(vesc)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)

ament_package()
