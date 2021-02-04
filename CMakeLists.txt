cmake_minimum_required(VERSION 2.8.3)
project(lidar_cam_calib_sdk)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)
add_compile_options(-fPIC)

# opencv
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_LIBRARIES})

# pcl
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# ceres 
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLDUE_DIRS})

# aurco mip
find_package(aruco REQUIRED)

include_directories(include/lidar_cam_calib_sdk)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
# lib
set(LIB_NAME "lidar2cam_calib")
set(incs
    include/lidar_cam_calib_sdk/lidar2cam_calib.h
    include/lidar_cam_calib_sdk/aruco_calib.h
    include/lidar_cam_calib_sdk/common.h
    include/lidar_cam_calib_sdk/laser_pattern.h
    include/lidar_cam_calib_sdk/mono_pattern.h
    include/lidar_cam_calib_sdk/prompt.h
    )
set(srcs
    src/common.cpp
    src/mono_pattern.cpp
    src/laser_pattern.cpp
    src/aruco_calib.cpp
    src/lidar2cam_calib.cpp
    )
add_library(${LIB_NAME} SHARED ${incs} ${srcs})
target_link_libraries(${LIB_NAME} 
                      ${PCL_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${CERES_LIBRARIES}
                      ${aruco_LIBS}
                      )

## catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()
add_executable(lidar2cam_calib_node node/lidar2cam_calib_node.cpp)
target_link_libraries(lidar2cam_calib_node ${LIB_NAME} ${catkin_LIBRARIES})

# message("source dir =  ${CMAKE_CURRENT_SOURCE_DIR}")

# copy header and library files
set(LIB_DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../lidar2cam_calib_release/lib/)
# add_custom_command(TARGET ${LIB_NAME} POST_BUILD
#                    COMMAND ${CMAKE_COMMAND} -E copy 
#                            $<TARGET_FILE:${LIB_NAME}>
#                            ${LIB_DESTINATION}
#                            )


### Test
add_executable(generate_marker tools/generate_marker.cpp)
target_link_libraries(generate_marker ${LIB_NAME} ${catkin_LIBRARIES})

