cmake_minimum_required(VERSION 3.8)
project(airtable_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Shared launch helpers: robot configuration loading and RViz generation.
ament_python_install_package(${PROJECT_NAME})

install(
    DIRECTORY
        launch
        rviz
        config
        foxglove
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/experiment_bag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME experiment_bag
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_disturbance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_disturbance
)
install(PROGRAMS scripts/robot_config.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_config
)
install(PROGRAMS scripts/valve_state.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME valve_state
)
install(PROGRAMS scripts/sun_direction.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sun_direction
)
install(PROGRAMS scripts/sim_sun_sensor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_sun_sensor
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_experiment_bag test/test_experiment_bag.py)
  ament_add_pytest_test(test_sim_disturbance test/test_sim_disturbance.py)
  ament_add_pytest_test(test_robot_config test/test_robot_config.py)
  ament_add_pytest_test(test_valve_state test/test_valve_state.py)
  ament_add_pytest_test(test_sun_direction test/test_sun_direction.py)
  ament_add_pytest_test(test_sim_sun_sensor test/test_sim_sun_sensor.py)
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
