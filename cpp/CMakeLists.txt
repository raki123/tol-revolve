# CMake Required Version
cmake_minimum_required (VERSION 2.8.12)

# Project Name
project (TriangleOfLife)

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

if (NOT ${REVOLVE_INCLUDE_PATH})
    # Add Revolve as an external project
    # TODO In the future this should fetch from a repository instead
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
endif()


# Find Boost and add to include dirs
find_package(Boost REQUIRED COMPONENTS system)

# Set include dirs and dependencies
include_directories(${Boost_INCLUDE_DIRS} ${REVOLVE_INCLUDE_PATH} /home/elte/mt/sdf_builder/src)
set(TOL_DEPS ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${REVOLVE_LIBRARIES})

# Gazebo control plugins
# Find and include Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})

# Plugin CPP files
file(GLOB_RECURSE
	 TOL_SRC
	 tol/*.cpp
)

add_library(tolmodelcontrol SHARED ${TOL_SRC})
target_link_libraries(tolmodelcontrol ${TOL_DEPS})
add_dependencies(tolmodelcontrol revolve)



