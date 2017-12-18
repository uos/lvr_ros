cmake_minimum_required(VERSION 2.4.6)

project(lvr_ros)

set(PACKAGE_DEPENDENCIES
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  genmsg
  mesh_msgs
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  geometry_msgs
  hyperspectral_calibration
  mbf_abstract_nav
  tf
  pcl_ros
  pcl_definitions
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(LVR REQUIRED)

add_definitions(${LVR_DEFINITIONS})

add_action_files(
  DIRECTORY
  action
  FILES
  Reconstruct.action
  SendCloud.action
  StartReconstruction.action
  StopReconstruction.action
)

find_path(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

find_package(PCL 1.2 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

find_package(OpenCL)
if(OPENCL_FOUND)
    message(STATUS "OpenCL Libraries: ${OpenCL_LIBRARIES}")
endif()

### compile with c++11
if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "--std=gnu++14 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 14)
endif ()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LVR_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/Reconstruction.cfg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  mesh_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  INCLUDE_DIRS include
  DEPENDS LVR
  LIBRARIES ${PROJECT_NAME}_conversions
)

add_library(${PROJECT_NAME}_conversions
  src/colors.cpp
  src/conversions.cpp
)

target_link_libraries(${PROJECT_NAME}_conversions
  ${catkin_LIBRARIES}
  ${LVR_LIBRARIES}
)

add_executable(${PROJECT_NAME}_reconstruction
  src/colors.cpp
  src/conversions.cpp
  src/reconstruction.cpp
)

target_link_libraries(${PROJECT_NAME}_reconstruction
    ${catkin_LIBRARIES}
    ${LVR_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_remote_reconstruction_client
    src/remote_reconstruction_client.cpp
)

add_executable(${PROJECT_NAME}_remote_reconstruction
  src/colors.cpp
  src/conversions.cpp
  src/remote_reconstruction.cpp
  src/FileObserver.cpp
)
target_link_libraries(${PROJECT_NAME}_remote_reconstruction
    ${catkin_LIBRARIES}
    ${LVR_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_remote_reconstruction_client
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
if(OPENCL_FOUND)
    target_compile_definitions(${PROJECT_NAME}_reconstruction PRIVATE OPENCL_FOUND=1)
endif()

add_executable(${PROJECT_NAME}_debug
    src/debug.cpp
)
target_link_libraries(${PROJECT_NAME}_debug
    ${catkin_LIBRARIES}
    ${LVR_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PROJECT_NAME}_conversions
)

add_dependencies(${PROJECT_NAME}_remote_reconstruction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_reconstruction
        ${lvr_ros_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
)
add_dependencies(${PROJECT_NAME}_conversions ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_debug ${catkin_EXPORTED_TARGETS})

install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS ${PROJECT_NAME}_conversions ${PROJECT_NAME}_reconstruction ${PROJECT_NAME}_debug
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
