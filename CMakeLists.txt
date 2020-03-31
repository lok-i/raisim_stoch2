cmake_minimum_required(VERSION 3.5)
project(raisimOgre_examples VERSION 0.2.0)

find_package(raisimOgre CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

#==
# Build Targets
#==

function(create_executable app_name file_name)
  add_executable(${app_name} ${file_name})

  target_link_libraries(${app_name} PUBLIC raisim::raisimOgre)
  
  target_include_directories(${app_name} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
  
  target_compile_definitions(${app_name} PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/rsc/")
endfunction()

# robots
create_executable(stoch2_benchmark Stoch2_walker_benchmark.cpp)
create_executable(stoch2 Stoch2_walker.cpp)
