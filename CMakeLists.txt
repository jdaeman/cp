cmake_minimum_required(VERSION 3.1)
project(mycp C CXX)

set(CMAKE_BUILD_TYPE Release)

find_package(ortools CONFIG REQUIRED)

add_executable(tsp src/tsp.cc)
target_include_directories(tsp PUBLIC ${ORTOOLS_INCLUDE_DIRS})
target_link_libraries(tsp ortools::ortools)

add_executable(cpisfun src/cp_is_fun.cc)
target_include_directories(cpisfun PUBLIC ${ORTOOLS_INCLUDE_DIRS})
target_link_libraries(cpisfun ortools::ortools)

add_executable(cpisfun2 src/cp_is_fun2.cc)
target_include_directories(cpisfun2 PUBLIC ${ORTOOLS_INCLUDE_DIRS})
target_link_libraries(cpisfun2 ortools::ortools)

add_executable(golomb src/golomb.cc)
target_include_directories(golomb PUBLIC ${ORTOOLS_INCLUDE_DIRS})
target_link_libraries(golomb ortools::ortools)
