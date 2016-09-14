# CMake Required Version
cmake_minimum_required (VERSION 2.8.12)

enable_testing()

# Project Name
project (TriangleOfLife)

# CMake flag to build to local directory
if (LOCAL_LIB_DIR)
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

# Use C++11 (removed, this is added below by the Gazebo flags)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Debug Flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG")

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")

##### Protobuf messages #####
# Get protobuf library
#find_package(Protobuf REQUIRED)

# Directory where the .proto file resides
#set(SPEC_DIR "${CMAKE_SOURCE_DIR}/tol/msgs")
#include_directories(${CMAKE_SOURCE_DIR} ${PROTOBUF_INCLUDE_DIRS})

# Include the binary dir since that is where the protobuf files are
# going to be placed.
#include_directories(${CMAKE_CURRENT_BINARY_DIR})
#file(GLOB_RECURSE REVOLVE_PROTO ${SPEC_DIR}/*.proto)
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${REVOLVE_PROTO})

# Generate messages library
#add_library(tol-msgs STATIC ${PROTO_SRCS} )
#target_link_libraries(tol-msgs ${PROTOBUF_LIBRARIES})

##### Plugins #####
# Find Boost and add to include dirs
find_package(Boost REQUIRED COMPONENTS system)

# Plugin CPP files
file(GLOB_RECURSE
    TOL_WORLD_SRC
    tol/plugin/WorldController.cpp
)

file(GLOB_RECURSE
    TOL_ROBOT_SRC
    tol/plugin/RobotController.cpp
    tol/plugin/fakebrain.cpp
    tol/plugin/evaluator.cpp
    tol/plugin/rlpower.cpp
    tol/plugin/supgbrain.cpp
    tol/plugin/neat/asyncneat.cpp
    tol/plugin/neat/neatevaluation.cpp
    tol/plugin/supg/supgneuron.cpp
    tol/plugin/supg/supggenomemanager.cpp
)

add_library(tolworldcontrol SHARED ${TOL_WORLD_SRC})
add_library(tolrobotcontrol SHARED ${TOL_ROBOT_SRC})

if (DEFINED REVOLVE_BUILD_PATH)
    message("Deriving include path / libary path from custom build path.")
    set(REVOLVE_INCLUDE_PATH ${REVOLVE_BUILD_PATH}/include)
    set(REVOLVE_LIBRARIES ${REVOLVE_BUILD_PATH}/lib/librevolve-gazebo.a ${REVOLVE_BUILD_PATH}/lib/librevolve-spec.a)
endif()

if (NOT DEFINED REVOLVE_INCLUDE_PATH)
    # Add Revolve as an external project
    # TODO In the future this should fetch from a repository instead
    message("Adding REVOLVE as external project.")
    message("To prevent this, pass REVOLVE_INCLUDE_PATH and REVOLVE_LIBRARIES or REVOLVE_BUILD_PATH.")
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
    set(REVOLVE_LIBRARIES ${INSTALL_DIR}/lib/librevolve-gazebo.a ${INSTALL_DIR}/lib/librevolve-spec.a)
    add_dependencies(tolmodelcontrol revolve)
endif()

### Include Gazebo as well
# Find and include Gazebo
# LOCAL_GAZEBO_DIR can be set to a path with a gazebo-config.cmake
if (LOCAL_GAZEBO_DIR)
    find_package(gazebo 6.5 REQUIRED
            PATHS "${LOCAL_GAZEBO_DIR}"
            NO_DEFAULT_PATH)
    message(WARNING "Using local Gazebo @ ${gazebo_DIR}")
else()
    find_package(gazebo 6.5 REQUIRED)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})

#### GSL ####
# Gnu mathematical library
find_package(GSL REQUIRED)

# Add Gazebo's C++ flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Need to include the Gazebo protobuf include path
set(GAZEBO_PROTOBUF_DIR)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_PROTO_PATH "${ITR}/gazebo/msgs/proto")
    set(GAZEBO_PROTO_INCLUDE "${ITR}/gazebo/msgs")
  endif()
endforeach()
include_directories(${GAZEBO_PROTO_INCLUDE})

# add accneat
add_subdirectory(tol/plugin/neat/accneat)
#TODO make include path for accneat nicer
include_directories(tol/plugin/neat/accneat/src)

# add revolve neat
add_subdirectory(../../revolve-brain/cpp revolve-brain)
include_directories(../../revolve-brain/cpp)

# Set include dirs and dependencies
include_directories(${Boost_INCLUDE_DIRS} ${REVOLVE_INCLUDE_PATH} ${GAZEBO_INCLUDE_DIRS} ${GSL_INCLUDE_DIRS})
set(TOL_DEPS ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${REVOLVE_LIBRARIES} ${GAZEBO_LIBRARIES} ${GSL_LIBRARIES})

target_link_libraries(tolrobotcontrol ${TOL_DEPS} accneat revolve-brain-static)
target_link_libraries(tolworldcontrol ${TOL_DEPS} tolrobotcontrol)

add_executable(testAsyncNeat tol/plugin/neat/test/testasyncneat.cpp)
add_executable(testCustomGenomeManager tol/plugin/neat/test/testcustomgenomemanager.cpp)
add_executable(testMultiNNSpecies tol/plugin/neat/test/testmultinnspeciesneat.cpp)
add_executable(testSUPGBrain tol/plugin/test/testsupgbrain.cpp)
target_link_libraries(testAsyncNeat tolrobotcontrol)
target_link_libraries(testCustomGenomeManager tolrobotcontrol)
target_link_libraries(testMultiNNSpecies tolrobotcontrol)
target_link_libraries(testSUPGBrain tolrobotcontrol)
add_test(testAsyncNeat testAsyncNeat)
add_test(testCustomGenomeManager testCustomGenomeManager)
add_test(testMultiNNSpecies testMultiNNSpecies)
add_test(testSUPGBrain testSUPGBrain)

# Evolution keys library
find_package(Qt4)
find_package(Protobuf REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
QT4_WRAP_CPP(headers_MOC tol/plugin/InteractiveEvolutionPlugin.h)
add_library(tolinteractiveevolution SHARED ${headers_MOC} tol/plugin/InteractiveEvolutionPlugin.cpp)

target_link_libraries(tolinteractiveevolution ${GAZEBO_LIBRARIES} ${QT_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_subdirectory(tol/plugin/test)
