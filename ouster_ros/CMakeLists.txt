cmake_minimum_required(VERSION 3.1.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             pcl_ros
             pcl_conversions
             roscpp
             tf2
             tf2_ros
             tf2_geometry_msgs
             cv_bridge)

# ==== Options ====
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Werror)
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

# ====  DBoW3 ===
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

# ==== Catkin ====
add_message_files(FILES PacketMsg.msg)
add_service_files(FILES OSConfigSrv.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

set(_ouster_ros_INCLUDE_DIRS "include;../ouster_client/include;../ouster_viz/include")

catkin_package(
  INCLUDE_DIRS
    ${_ouster_ros_INCLUDE_DIRS}
  LIBRARIES
    ouster_ros
  CATKIN_DEPENDS
    roscpp
    message_runtime
    pcl_ros
    std_msgs
    sensor_msgs
    geometry_msgs
  DEPENDS
    EIGEN3 GLFW3 GLEW)

# ==== Libraries ====
# Build static libraries and bundle them into ouster_ros using the `--whole-archive` flag. This is
# necessary because catkin doesn't interoperate easily with target-based cmake builds. Object
# libraries are the recommended way to do this, but require >=3.13 to propagate usage requirements.
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(../ouster_client ouster_client EXCLUDE_FROM_ALL)
add_subdirectory(../ouster_viz ouster_viz EXCLUDE_FROM_ALL)
set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(${_ouster_ros_INCLUDE_DIRS} SYSTEM ${catkin_INCLUDE_DIRS})

add_library(ouster_ros src/ros.cpp)
target_link_libraries(ouster_ros PUBLIC ${catkin_LIBRARIES} ouster_build PRIVATE
  -Wl,--whole-archive ouster_client ouster_viz -Wl,--no-whole-archive)
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

# ==== Executables ====
add_executable(os_node src/os_node.cpp)
target_link_libraries(os_node ouster_ros ${catkin_LIBRARIES})
add_dependencies(os_node ${PROJECT_NAME}_gencpp)

add_executable(os_cloud_node src/os_cloud_node.cpp)
target_link_libraries(os_cloud_node ouster_ros ${catkin_LIBRARIES})
add_dependencies(os_cloud_node ${PROJECT_NAME}_gencpp)

add_executable(viz_node src/viz_node.cpp)
target_link_libraries(viz_node ouster_ros ${catkin_LIBRARIES} glfw GLEW)
add_dependencies(viz_node ${PROJECT_NAME}_gencpp)

add_executable(img_node src/img_node.cpp)
target_link_libraries(img_node ouster_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} ${DBoW3_LIBS})
add_dependencies(img_node ${PROJECT_NAME}_gencpp)

# ==== Install ====
install(
  TARGETS ouster_ros os_node os_cloud_node viz_node img_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(
  DIRECTORY ${_ouster_ros_INCLUDE_DIRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES ouster.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
