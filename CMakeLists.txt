cmake_minimum_required(VERSION 3.8)
project(go2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

# Add other necessary find_package calls
find_package(rosidl_default_generators REQUIRED)

install(
  DIRECTORY config dae launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

