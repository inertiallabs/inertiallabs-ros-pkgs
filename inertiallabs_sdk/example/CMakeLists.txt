cmake_minimum_required(VERSION 3.11)
project(ilsdk_examples LANGUAGES CXX)

if(NOT TARGET inertiallabs_sdk)
    # Stand-alone build
    find_package(InertialLabsSdk REQUIRED)
endif()

add_executable(example1 example.cpp)
target_link_libraries(example1 PRIVATE inertiallabs_sdk)
if (WIN32)
    install(FILES $<TARGET_RUNTIME_DLLS:example1> TYPE BIN)
    add_custom_command( TARGET example1 POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_RUNTIME_DLLS:example1>
            $<TARGET_FILE_DIR:example1>
            COMMAND_EXPAND_LISTS)
endif()