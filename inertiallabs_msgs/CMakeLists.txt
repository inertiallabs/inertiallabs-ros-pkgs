cmake_minimum_required(VERSION 2.8.3)
project(inertiallabs_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs actionlib_msgs message_generation)

add_message_files(
    DIRECTORY msg
    FILES
    ins_data.msg
    sensor_data.msg
    gnss_data.msg
    gps_data.msg
    marine_data.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    actionlib_msgs
    message_runtime
)
