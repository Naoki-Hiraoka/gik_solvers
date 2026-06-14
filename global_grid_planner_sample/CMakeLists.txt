cmake_minimum_required(VERSION 3.0.2)
project(global_grid_planner_sample)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  ik_constraint2_bullet
  global_grid_planner
  choreonoid_viewer
  )
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidGlobalGridPlannerSamplePlugin SHARED
  src/CnoidGlobalGridPlannerSamplePlugin.cpp
  src/sample1.cpp
  src/sample2.cpp
  )

target_link_libraries(CnoidGlobalGridPlannerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidGlobalGridPlannerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
