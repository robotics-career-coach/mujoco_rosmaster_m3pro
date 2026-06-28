cmake_minimum_required(VERSION 3.8)
project(m3pro_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  mjcf
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mjcf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
