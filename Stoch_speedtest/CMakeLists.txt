cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories (${EIGEN3_INCLUDE_DIRS})

add_executable(raisim_frequency speed_test.cpp)
target_link_libraries(raisim_frequency PUBLIC raisim::raisim)
target_include_directories(raisim_frequency PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
