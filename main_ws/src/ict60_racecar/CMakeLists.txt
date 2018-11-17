cmake_minimum_required(VERSION 2.8.3)
project(ict60_racecar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES api_lane_detection
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(api_lane_detection
  src/car_control.cpp
  src/lane_detector.cpp
)

add_executable(racecar src/racecar.cpp src/traffic_sign.cpp)
add_executable(manual_control src/manual_control.cpp)

add_dependencies(racecar ${catkin_EXPORTED_TARGETS})
add_dependencies(manual_control ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  racecar
  api_lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
target_link_libraries(
  manual_control
  api_lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

install(TARGETS api_lane_detection ${ADDITIONAL_EXECUTABLES}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
