# CMake Required Version
cmake_minimum_required (VERSION 2.8.12)

# Project Name
project (TriangleOfLife)

# CMake flag to build to local directory
if (${LOCAL_LIB_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../build")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../build")
    message("Using local library directory: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
endif()

# Pass source dir to preprocessor
add_definitions(-DSOURCE_DIR=${CMAKE_SOURCE_DIR})

# Compiler options
# TODO This currently assumes GCC, add Windows support in due time
add_definitions(-pedantic -Wno-long-long -Wall -Wextra -Wformat=2
                -Wredundant-decls -Wwrite-strings -Wmissing-include-dirs 
                -Wswitch-enum -Wuninitialized 
                -Wswitch-default -Winit-self -Wfloat-equal -fPIC )

# Use C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Debug Flags 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG")

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")

##### Protobuf messages #####
# Get protobuf library
find_package(Protobuf REQUIRED)

# Directory where the .proto file resides
set(SPEC_DIR "${CMAKE_SOURCE_DIR}/tol/msgs")
include_directories(${CMAKE_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS})

# Include the binary dir since that is where the protobuf files are
# going to be placed.
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB_RECURSE REVOLVE_PROTO ${SPEC_DIR}/*.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${REVOLVE_PROTO})

# Generate messages library
add_library(tol-msgs STATIC ${PROTO_SRCS} )
target_link_libraries(tol-msgs ${PROTOBUF_LIBRARIES})

##### Plugins #####
# Find Boost and add to include dirs
find_package(Boost REQUIRED COMPONENTS system)

# Gazebo control plugins
# Find and include Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})

# Plugin CPP files
file(GLOB_RECURSE
	 TOL_WORLD_SRC
	 tol/plugin/WorldController.cpp
)

file(GLOB_RECURSE
	 TOL_ROBOT_SRC
	 tol/plugin/RobotController.cpp
)

add_library(tolworldcontrol SHARED ${TOL_WORLD_SRC})
add_library(tolrobotcontrol SHARED ${TOL_ROBOT_SRC})

if (NOT DEFINED REVOLVE_INCLUDE_PATH)
    # Add Revolve as an external project
    # TODO In the future this should fetch from a repository instead
    message("Adding REVOLVE as external project.")
    message("To prevent this, pass REVOLVE_INCLUDE_PATH and REVOLVE_LIBRARIES.")
    include(ExternalProject)
    ExternalProject_Add(
      revolve
      SOURCE_DIR ${CMAKE_SOURCE_DIR}/../../revolve/cpp
      INSTALL_DIR revolve
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/revolve
    )

    # Get Revolve lib / include directories
    ExternalProject_Get_Property(revolve INSTALL_DIR)
    set(REVOLVE_INCLUDE_PATH ${INSTALL_DIR}/include)
    set(REVOLVE_LIBRARIES ${INSTALL_DIR}/lib/librevolve-gazebo.a)
    add_dependencies(tolmodelcontrol revolve)
endif()

# Set include dirs and dependencies
include_directories(${Boost_INCLUDE_DIRS} ${REVOLVE_INCLUDE_PATH})
set(TOL_DEPS ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${REVOLVE_LIBRARIES})

target_link_libraries(tolworldcontrol tol-msgs ${TOL_DEPS})
target_link_libraries(tolrobotcontrol tol-msgs ${TOL_DEPS})


