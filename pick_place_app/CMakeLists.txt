cmake_minimum_required(VERSION 3.8)
project(pick_place_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosparam_shortcuts REQUIRED)
find_package(pick_place_msgs REQUIRED)

set(pick_place_dependencies
  ament_cmake
  rclcpp
  moveit_task_constructor_core
  Boost
  pluginlib
  rclcpp_components
  rosparam_shortcuts
)

set(pick_place_server_dependencies
  ament_cmake
  rclcpp
  moveit_task_constructor_core
  Boost
  pluginlib
  rclcpp_components
  rosparam_shortcuts
  pick_place_msgs
)

# System dependencies are found with CMake's conventions
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ##########
# # Build pick place demo ##
# ##########
set(pick_place_executable pick_place_demo)
set(pick_place_task_lib ${pick_place_executable}_core)

add_library(${pick_place_task_lib} SHARED
  src/pick_place_task.cpp)
ament_target_dependencies(${pick_place_task_lib}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  ${pick_place_dependencies})

add_executable(${pick_place_executable} src/main_pick_place.cpp)
target_compile_options(${pick_place_executable} PUBLIC -g -O0)
target_link_libraries(${pick_place_executable}
  ${pick_place_task_lib})
target_include_directories(${pick_place_executable}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ##########
# # Build pick place server ##
# ##########
set(pick_place_server_executable pick_place_server)
set(pick_place_server_lib ${pick_place_server_executable}_core)

add_library(${pick_place_server_lib} SHARED
  src/pick_place_server.cpp
  src/pick_place_task.cpp)
ament_target_dependencies(
  ${pick_place_server_lib}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  ${pick_place_server_dependencies})

add_executable(${pick_place_server_executable} src/main_pick_place_server.cpp)
target_compile_options(${pick_place_server_executable} PUBLIC -g -O0)
target_link_libraries(${pick_place_server_executable}
  ${pick_place_server_lib})
target_include_directories(${pick_place_server_executable}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${pick_place_task_lib}
  ${pick_place_executable}
  ${pick_place_server_lib}
  ${pick_place_server_executable}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${pick_place_task_lib}
)
ament_export_dependencies(${pick_place_dependencies})
ament_package()
