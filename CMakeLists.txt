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
  label_manager
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(LVR2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(MPI REQUIRED)

add_definitions(${LVR2_DEFINITIONS} ${OpenCV_DEFINITIONS})

add_action_files(
  DIRECTORY
  action
  FILES
  Reconstruct.action
)

find_path(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

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

# enable openmp support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LVR2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
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
  DEPENDS LVR2 MPI
  LIBRARIES ${PROJECT_NAME}_conversions
)

add_library(${PROJECT_NAME}_conversions
  src/colors.cpp
  src/conversions.cpp
)

target_link_libraries(${PROJECT_NAME}_conversions
  ${catkin_LIBRARIES}
  ${LVR2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)

add_executable(${PROJECT_NAME}_reconstruction
  src/colors.cpp
  src/conversions.cpp
  src/reconstruction.cpp
)

target_link_libraries(${PROJECT_NAME}_reconstruction
  ${catkin_LIBRARIES}
  ${LVR2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)

if(OPENCL_FOUND)
  target_compile_definitions(${PROJECT_NAME}_reconstruction PRIVATE OPENCL_FOUND=1)
endif()

# HDF5 to message executable
# link libraries
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
include_directories(${HDF5_INCLUDE_DIRS})

add_dependencies(${PROJECT_NAME}_reconstruction
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_gencpp
  )

add_dependencies(${PROJECT_NAME}_conversions
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_gencpp
)

install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS ${PROJECT_NAME}_conversions ${PROJECT_NAME}_reconstruction
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

message("LVR2 LIBRARIES " ${LVR2_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
