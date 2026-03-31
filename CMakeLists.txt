cmake_minimum_required(VERSION 3.8)
project(peak_cam)

# Default to C++17 (required for std::clamp and modern ROS 2 conventions)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ids_peak REQUIRED)
find_package(ids_peak_ipl REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

# samples for ids peak library are usually installed to /usr/local/src/ids/samples/peak
set(IDS_PEAK_INCLUDE_HINTS "")
if(DEFINED IDS_PEAK_INCLUDE_DIR)
  list(APPEND IDS_PEAK_INCLUDE_HINTS ${IDS_PEAK_INCLUDE_DIR})
endif()
if(DEFINED IDS_PEAK_IPL_INCLUDE_DIR)
  list(APPEND IDS_PEAK_INCLUDE_HINTS ${IDS_PEAK_IPL_INCLUDE_DIR})
endif()
if(DEFINED ids_peak_INCLUDES)
  list(APPEND IDS_PEAK_INCLUDE_HINTS ${ids_peak_INCLUDES})
endif()
if(DEFINED ids_peak_ipl_INCLUDES)
  list(APPEND IDS_PEAK_INCLUDE_HINTS ${ids_peak_ipl_INCLUDES})
endif()
list(REMOVE_DUPLICATES IDS_PEAK_INCLUDE_HINTS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/peak_cam_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${IDS_PEAK_INCLUDE_HINTS}
)

if(TARGET ids_peak)
  target_link_libraries(${PROJECT_NAME} ids_peak)
elseif(DEFINED IDS_PEAK_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} ${IDS_PEAK_LIBRARIES})
endif()

if(TARGET ids_peak_ipl)
  target_link_libraries(${PROJECT_NAME} ids_peak_ipl)
elseif(DEFINED IDS_PEAK_IPL_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} ${IDS_PEAK_IPL_LIBRARIES})
endif()


target_compile_definitions(${PROJECT_NAME} PRIVATE "PEAK_CAM_BUILDING_DLL")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "peak_cam::PeakCamNode"
  EXECUTABLE peak_cam_node
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/params
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)

ament_auto_package()
