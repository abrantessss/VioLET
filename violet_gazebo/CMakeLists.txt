cmake_minimum_required(VERSION 3.8)
project(violet_gazebo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(gazebo REQUIRED)

add_library(violet_rigid_joint_world_plugin SHARED
  src/rigid_joint_world_plugin.cpp
)
target_include_directories(violet_rigid_joint_world_plugin SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(violet_rigid_joint_world_plugin ${GAZEBO_LIBRARIES})
target_compile_features(violet_rigid_joint_world_plugin PUBLIC cxx_std_17)

add_library(violet_gimbal_joint_world_plugin SHARED
  src/gimbal_joint_world_plugin.cpp
)
target_include_directories(violet_gimbal_joint_world_plugin SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(violet_gimbal_joint_world_plugin ${GAZEBO_LIBRARIES})
target_compile_features(violet_gimbal_joint_world_plugin PUBLIC cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  violet_rigid_joint_world_plugin
  violet_gimbal_joint_world_plugin
  LIBRARY DESTINATION lib
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_auto_package()
