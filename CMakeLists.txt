cmake_minimum_required(VERSION 3.1)
project(mycp C CXX)

set(CMAKE_BUILD_TYPE Release)

find_package(ortools CONFIG REQUIRED)

add_executable(tsp src/tsp.cc)
target_include_directories(tsp PUBLIC ${ORTOOLS_INCLUDE_DIRS})
target_link_libraries(tsp ortools::ortools)
