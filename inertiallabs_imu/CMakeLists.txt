cmake_minimum_required(VERSION 2.8.3)
project(inertiallabs_imu)

find_package(catkin REQUIRED COMPONENTS
tf
rospy 
roscpp
geometry_msgs
sensor_msgs
std_msgs
dynamic_reconfigure
actionlib_msgs
inertiallabs_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV)

set(DEPRECATION_FLAG "-Wsizeof-array-argument")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra ${DEPRECATION_FLAG}")

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES inertiallabs_imu
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs tf dynamic_reconfigure  actionlib_msgs
  DEPENDS system_lib
)

include_directories(${Opencv_INCLUDE_DIRS})
include_directories(${system_lib_INCLUDE_DIRS})

include_directories(../inertiallabs_sdk/include)


add_library(IntertialLabs_IMU ../inertiallabs_sdk/src/IntertialLabs_IMU.c)
add_library(InertialLabs_services ../inertiallabs_sdk/src/arch/linux/InertialLabs_services.c)

target_link_libraries(InertialLabs_services pthread)


add_executable(imu_continues_mode  src/imu_continues_mode.cpp
                         ../inertiallabs_sdk/src/arch/linux/InertialLabs_services.c
                         ../inertiallabs_sdk/src/IntertialLabs_IMU.c
)

#target_link_libraries(InertialLabs_services pthread)

target_link_libraries(imu_continues_mode ${catkin_LIBRARIES} InertialLabs_services IntertialLabs_IMU)


add_executable(imu_onrequest_mode  src/imu_onrequest_mode.cpp
                         ../inertiallabs_sdk/src/arch/linux/InertialLabs_services.c
                         ../inertiallabs_sdk/src/IntertialLabs_IMU.c
)

add_dependencies(imu_onrequest_mode inertiallabs_imu_generate_messages_cpp)

#target_link_libraries(InertialLabs_services pthread)

target_link_libraries(imu_onrequest_mode ${catkin_LIBRARIES} InertialLabs_services IntertialLabs_IMU)
