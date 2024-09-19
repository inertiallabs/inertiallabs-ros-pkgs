cmake_minimum_required(VERSION 3.5)
project(inertiallabs_sdk VERSION 1.0.0.0)

include(GNUInstallDirs)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
option(ILSDK_BUILD_EXAMPLES "Build examples sources" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (WIN32)
    set(IO_SOURCES src/platform/windows/NetClient.cpp src/platform/windows/SerialPort.cpp)
elseif (UNIX)
    set(IO_SOURCES src/platform/linux/NetClient.cpp src/platform/linux/SerialPort.cpp)
else ()
    set(IO_SOURCES)
endif ()

add_library(inertiallabs_sdk
        ${IO_SOURCES}
        src/ILDriver.cpp
        src/UDDParser.cpp
)
target_include_directories(inertiallabs_sdk PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/inertiallabs_sdk>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_include_directories(inertiallabs_sdk PRIVATE src)
if(WIN32)
    if (BUILD_SHARED_LIBS)
        target_compile_definitions(inertiallabs_sdk PRIVATE ILSDK_EXPORT INTERFACE ILSDK_SHARED)
    endif()
endif()
set_target_properties(inertiallabs_sdk PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})

if(ILSDK_BUILD_EXAMPLES)
    message(STATUS "Generating example(s)")
    add_subdirectory(example)
endif()

install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(TARGETS inertiallabs_sdk
        EXPORT InertialLabsSdkTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(EXPORT InertialLabsSdkTargets
        FILE InertialLabsSdkTargets.cmake
        NAMESPACE InertialLabsSdk::
        DESTINATION lib/cmake/InertialLabsSdk
)
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/InertialLabsSdkConfig.cmake"
        INSTALL_DESTINATION "lib/cmake/InertialLabsSdk"
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
# generate the version file for the config file
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/InertialLabsSdkConfigVersion.cmake"
        VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
        COMPATIBILITY AnyNewerVersion
)

# install the generated configuration files
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/InertialLabsSdkConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/InertialLabsSdkConfigVersion.cmake
        DESTINATION lib/cmake/InertialLabsSdk
)
# generate the export targets for the build tree
# needs to be after the install(TARGETS) command
export(EXPORT InertialLabsSdkTargets
        FILE "${CMAKE_CURRENT_BINARY_DIR}/InertialLabsSdkTargets.cmake"
)
