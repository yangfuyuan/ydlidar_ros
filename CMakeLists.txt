cmake_minimum_required(VERSION 2.8.3)
project(ydlidar)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-std=c++11) # Use C++11

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

#add_subdirectory(sdk)

set(SDK_PATH "./ydlidar_sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/Lidar/Drivers/YDLidar/*.cpp"
  "${SDK_PATH}/Serial/impl/unix/*.cpp"
  "${SDK_PATH}/Serial/*.cpp"
  "${SDK_PATH}/Serial/*.c"
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/ydlidar_sdk/
  ${PROJECT_SOURCE_DIR}/ydlidar_sdk/Lidar
  ${PROJECT_SOURCE_DIR}/ydlidar_sdk/Lidar/Drivers
  ${PROJECT_SOURCE_DIR}/ydlidar_sdk/Serial
)

add_executable(ydlidar_node src/ydlidar_node.cpp ${SDK_SRC})
add_executable(ydlidar_client src/ydlidar_client.cpp)

target_link_libraries(ydlidar_node
   ${catkin_LIBRARIES} 
 )
target_link_libraries(ydlidar_client
   ${catkin_LIBRARIES} 
 )

install(TARGETS ydlidar_node ydlidar_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch startup ydlidar_sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


