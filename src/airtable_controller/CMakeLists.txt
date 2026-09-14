cmake_minimum_required(VERSION 3.8)
project(airtable_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(airtable_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(
    airtable_controller_node
    src/controller_node.cpp
    src/control_allocator.cpp
    src/manual_mixer.cpp
)

target_include_directories(
    airtable_controller_node
    PRIVATE
        include
)

ament_target_dependencies(
    airtable_controller_node
    rclcpp
    geometry_msgs
    airtable_interfaces
)

target_link_libraries(
    airtable_controller_node
    Eigen3::Eigen
)

install(
    TARGETS
        airtable_controller_node
    DESTINATION 
        lib/${PROJECT_NAME}
)

install(
    DIRECTORY 
        config
    DESTINATION share/${PROJECT_NAME}
)

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
