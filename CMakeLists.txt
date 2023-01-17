cmake_minimum_required(VERSION 3.25)
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
    INSTALL_COMMAND ""
)
FetchContent_Declare(data_structure
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/data-structure-aacal.git
    GIT_TAG master
    TIMEOUT 5
    INSTALL_COMMAND ""
)
FetchContent_Declare(
    read_parameter_struct
    GIT_REPOSITORY https://gitlab.kuleuven.be/u0141557/read_parameter_struct.git
    GIT_TAG 17a40e2fd35c53be45c7500117f38e3ce7901e40
    FIND_PACKAGE_ARGS 
)
FetchContent_MakeAvailable(five_c data_structure read_parameter_struct)

if(NOT ${five_c_FOUND})
    set(FIVE_C_INCLUDE_DIRS ${five_c_SOURCE_DIR}/include)
    set(FIVE_C_LIBRARIES ${five_c_BINARY_DIR}/libfive_c.so)
endif()

if(NOT ${data_structure_FOUND})
    set(DATA_STRUCTURE_INCLUDE_DIRS ${data_structure_SOURCE_DIR}/include)
    set(DATA_STRUCTURE_LIBRARIES ${data_structure_BINARY_DIR}/libdata_structure.so)
endif()

if(NOT ${read_parameter_struct_FOUND})
    set(READ_PARAMETER_STRUCT_INCLUDE_DIRS ${read_parameter_struct_SOURCE_DIR}/include)
    set(READ_PARAMETER_STRUCT_LIBRARIES ${read_parameter_struct_BINARY_DIR}/libread_parameter_struct.so)
endif()


##########################
##        Build         ##
##########################
add_library(${PROJECT_NAME} SHARED
    src/basic.c
    src/motion_primitive.c
    src/free_space.c
    #src/free_space_activity.c
)
add_dependencies(${PROJECT_NAME} five_c data_structure read_parameter_struct)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${DATA_STRUCTURE_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${READ_PARAMETER_STRUCT_INCLUDE_DIRS}>
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        m
        ${FIVE_C_LIBRARIES}
        ${DATA_STRUCTURE_LIBRARIES}
        ${READ_PARAMETER_STRUCT_LIBRARIES}
)


# # UNIT TESTS
if(BUILD_GTESTS)
    add_subdirectory(test)
endif()


# #add_executable(main src/main.c)
# #target_link_libraries(main PUBLIC
# #    aaca-lib
# #    ${PROJECT_NAME}
# #    pthread
# #)

# add_executable(marloc src/marloc.c)
# target_link_libraries(marloc PUBLIC
#     ${PROJECT_NAME}
#     aaca-lib
#     pthread
# )
