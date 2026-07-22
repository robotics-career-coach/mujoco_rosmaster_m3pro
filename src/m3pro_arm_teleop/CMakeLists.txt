cmake_minimum_required(VERSION 3.8)
project(m3pro_arm_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/arm_teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
