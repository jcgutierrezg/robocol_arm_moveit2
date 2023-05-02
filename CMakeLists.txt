cmake_minimum_required(VERSION 3.8)
# The argument to project will be the package name and must be identical to the package name in the package.xml.
project(robocol_arm_moveit2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# If a package is found by CMake through find_package, it results in the creation of several CMake environment variables that give information about the found package. 
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Add your nodes
add_executable(main_node src/main_node.cpp)
add_executable(planning_test src/planning_test.cpp)

# This adds all files in the folder ${CMAKE_CURRENT_SOURCE_DIR}/include to the public interface during build time and all files in the include folder (relative to ${CMAKE_INSTALL_DIR}) when being installed.
target_include_directories(main_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(planning_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# Specification to indicate that the package requires C99 and C++17
target_compile_features(main_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(planning_test PUBLIC c_std_99 cxx_std_17)

# It includes the necessary headers and libraries and their dependencies to be correctly found by the project. It will also ensure that the include directories of all dependencies are ordered correctly when using overlay workspaces.
ament_target_dependencies(main_node moveit_ros_planning_interface "rclcpp")
ament_target_dependencies(planning_test "moveit_ros_planning_interface" "rclcpp")

# Install executables
install(TARGETS main_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS planning_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})

# Files for testing purposes
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

# The project setup is done by ament_package() and this call must occur exactly once per package. ament_package() installs the package.xml, registers the package with the ament index, and installs config (and possibly target) files for CMake so that it can be found by other packages using find_package.
ament_package()
