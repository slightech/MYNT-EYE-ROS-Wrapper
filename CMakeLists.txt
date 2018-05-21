cmake_minimum_required(VERSION 2.8.3)
project(mynteye_ros_wrapper)

add_definitions(-std=c++11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(DEFINED ENV{MYNTEYE_SDK_ROOT})
  set(MYNTEYE_SDK_DIR $ENV{MYNTEYE_SDK_ROOT})
else()
  message(FATAL_ERROR "MYNTEYE_SDK_ROOT environment variable not found.")
endif()
set(MYNT_SDK_LIB_DIR ${MYNTEYE_SDK_DIR}/lib)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  pcl_conversions
  rosconsole
  roscpp
  sensor_msgs
  tf2_ros
  nodelet
)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  if(OpenCV_VERSION VERSION_LESS 3.0)
    set(USE_OPENCV2 TRUE)
    add_definitions(-DUSE_OPENCV2)
  else()
    set(USE_OPENCV3 TRUE)
    add_definitions(-DUSE_OPENCV3)
  endif()
else()
  message(FATAL_ERROR "OpenCV not found. Allow set OpenCV_DIR to find it.")
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${MYNTEYE_SDK_DIR}/include
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${MYNT_SDK_LIB_DIR}
)

# targets

add_library(MYNTWrapper src/mynteye_ros_wrapper_nodelet.cpp)
target_link_libraries(MYNTWrapper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${MYNT_SDK_LIB_DIR}/libmynteye_core.so)
add_dependencies(MYNTWrapper ${PROJECT_NAME}_gencfg)

add_executable(mynteye_wrapper_node src/mynteye_ros_wrapper_node.cpp)
target_link_libraries(mynteye_wrapper_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  MYNTWrapper
)
add_dependencies(mynteye_wrapper_node ${PROJECT_NAME}_gencfg)

# install

FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

install(TARGETS
  MYNTWrapper
  mynteye_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
