cmake_minimum_required(VERSION 3.5)
project(peak_cam)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/params
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)

ament_auto_package()
