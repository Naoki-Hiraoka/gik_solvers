cmake_minimum_required(VERSION 3.0.2)
project(global_inverse_kinematics_solver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  prioritized_inverse_kinematics_solver2
  ompl_near_projection
  choreonoid_viewer
  )

find_package(choreonoid REQUIRED)
find_package(ompl REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_inverse_kinematics_solver
  CATKIN_DEPENDS ompl_near_projection prioritized_inverse_kinematics_solver2
  )

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
)

link_directories(
 ${OMPL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/global_inverse_kinematics_solver.cpp
  src/GIKConstraint.cpp
  src/GIKStateSpace.cpp
  src/GIKGoalSpace.cpp
  src/CnoidStateSpace.cpp
  src/GIKProjectionEvaluator.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${OMPL_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
