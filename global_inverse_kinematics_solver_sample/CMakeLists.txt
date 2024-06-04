cmake_minimum_required(VERSION 3.0.2)
project(global_inverse_kinematics_solver_sample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  ik_constraint2_vclip
  ik_constraint2_bullet
  ik_constraint2_distance_field
  choreonoid_cddlib
  global_inverse_kinematics_solver
  roslib
  choreonoid_viewer
  )
find_package(choreonoid REQUIRED)
find_package(ompl REQUIRED)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
)

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

add_executable(SampleSR1
  src/SampleSR1.cpp
)

target_link_libraries(SampleSR1
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  ${OMPL_LIBRARIES}
  )

install(TARGETS SampleSR1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )



add_library(CnoidGlobalInverseKinematicsSolverSamplePlugin SHARED
  src/CnoidGlobalInverseKinematicsSolverSamplePlugin.cpp
  src/sample1_4limb.cpp
  src/sample2_desk.cpp
  src/sample3_desk.cpp
  src/sample4_jaxon.cpp
  src/sample5_jaxon.cpp
  src/samplerobot_common.cpp
  src/sample6_root.cpp
  src/sample7_rootrej.cpp
)
target_link_libraries(CnoidGlobalInverseKinematicsSolverSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  )
set_target_properties(CnoidGlobalInverseKinematicsSolverSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
