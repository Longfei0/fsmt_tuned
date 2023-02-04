cmake_minimum_required(VERSION 3.15)
set(PROJECT_VERSION "0.0.2")
set(CMAKE_CXX_STANDARD 11)
project(free_space_motion_tube VERSION ${PROJECT_VERSION} LANGUAGES C CXX)

set(FREE_SPACE_MOTION_TUBE_VERSION_SO 0)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE ALWAYS)

##########################
##         Flags        ##
##########################
option(BUILD_GTESTS "Build unit tests" OFF)
option(BUILD_5C_ACTIVITY "Build free space motion tube activity (5C)" ON)

##########################
##     Dependencies     ##
##########################
set(five_c_FOUND false)
set(data_structure_FOUND false)
set(read_parameter_struct_FOUND false)

include(FetchContent)
FetchContent_Declare(five_c
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/algorithm_and_activity_component_architecture.git
    GIT_TAG master
    TIMEOUT 5
)
FetchContent_Declare(read_parameter_struct
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0141557/read_parameter_struct.git
    GIT_TAG main
    TIMEOUT 5
)
FetchContent_Declare(data_structure
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/data-structure-aacal.git
    GIT_TAG master
    TIMEOUT 5
)

FetchContent_MakeAvailable(data_structure)

if(NOT ${data_structure_FOUND})
    set(DATA_STRUCTURE_INCLUDE_DIRS ${data_structure_SOURCE_DIR}/include)
    set(DATA_STRUCTURE_BUILD_DIRS ${data_structure_BINARY_DIR})
    set(DATA_STRUCTURE_LIBRARIES ${data_structure_BINARY_DIR}/libdata_structure.so)
endif()

##########################
##        Build         ##
##########################
add_library(${PROJECT_NAME} SHARED
    src/core/basic.c
    src/core/motion_primitive.c
#    src/core/free_space.c
    src/core/motion_tube.c
    src/core/motion_tube_cartesian.c
    src/core/motion_tube_sensor_space.c
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${DATA_STRUCTURE_INCLUDE_DIRS}>
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        m
)

##########################
##  Build (Optionals)   ##
##########################

if(BUILD_GTESTS)
    add_subdirectory(test)
endif()

#if(BUILD_5C_ACTIVITY)
#    add_subdirectory(src/activity)
#endif()

