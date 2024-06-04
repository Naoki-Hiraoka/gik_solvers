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

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

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
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  ${OMPL_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
