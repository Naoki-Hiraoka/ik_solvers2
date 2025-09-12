cmake_minimum_required(VERSION 3.0.2)
project(prioritized_inverse_kinematics_solver2_sample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  ik_constraint2_vclip
  ik_constraint2_bullet
  ik_constraint2_distance_field
  ik_constraint2_region_cdd
  choreonoid_cddlib
  prioritized_inverse_kinematics_solver2
  roslib
  choreonoid_viewer
  )
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_executable(SampleSR1
  src/SampleSR1.cpp
)

target_link_libraries(SampleSR1
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )

install(TARGETS SampleSR1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )



add_library(CnoidPrioritizedInverseKinematicsSolver2SamplePlugin SHARED
  src/CnoidPrioritizedInverseKinematicsSolver2SamplePlugin.cpp
  src/sample0_1_bullet.cpp
  src/sample0_2_bullet.cpp
  src/sample1_4limb.cpp
  src/sample2_collision.cpp
  src/sample3_collision.cpp
  src/sample4_region.cpp
  src/sample5_or.cpp
  src/sample6_keep.cpp
  src/sample7_keep.cpp
)
target_link_libraries(CnoidPrioritizedInverseKinematicsSolver2SamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidPrioritizedInverseKinematicsSolver2SamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
