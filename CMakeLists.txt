cmake_minimum_required(VERSION 3.8)
project(ntarobot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Required python package
ament_python_install_package(${PROJECT_NAME})

# Install python nodes made by python
# Required to make python and c++ integration
install(
  PROGRAMS
  ${PROJECT_NAME}/ntarobot_motordriver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalacion de paquetes agregados
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
