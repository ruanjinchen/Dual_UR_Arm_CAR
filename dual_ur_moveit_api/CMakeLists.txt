cmake_minimum_required(VERSION 3.8)
project(dual_ur_moveit_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(dual_ur_moveit_api src/api_control.cpp src/halcon.cpp)
target_include_directories(dual_ur_moveit_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dual_ur_moveit_api PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dual_ur_moveit_api
  "moveit_ros_planning_interface"
  "rclcpp"
)

#MUST!!!
target_link_libraries(dual_ur_moveit_api
  ${catkin_LIBRARIES}
  jsoncpp
)

install(TARGETS dual_ur_moveit_api
  DESTINATION lib/${PROJECT_NAME})

#MUST!!!
install(DIRECTORY src launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
