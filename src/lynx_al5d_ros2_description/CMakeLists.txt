cmake_minimum_required(VERSION 3.8)
project(lynx_al5d_ros2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ESTA ES LA PARTE IMPORTANTE QUE TE FALTA
install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
