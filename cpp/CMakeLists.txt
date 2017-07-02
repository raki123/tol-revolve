# Initial setups
# _____________________________________________________________________________

# CMake Required Version
cmake_minimum_required(VERSION 2.8.12)

enable_testing()

# Project Name
project(TriangleOfLife)

# CMake flag to build to local directory
if (LOCAL_LIB_DIR)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../build")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../build")
  message("Using local library directory: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
endif ()

# Pass source dir to preprocessor
add_definitions(-DSOURCE_DIR=${CMAKE_SOURCE_DIR})

# Compiler options
# TODO This currently assumes GCC, add Windows support in due time
add_definitions(-pedantic -Wno-long-long -Wall -Wextra -Wformat=2
                -Wredundant-decls -Wwrite-strings -Wmissing-include-dirs
                -Wswitch-enum -Wuninitialized
                -Wswitch-default -Winit-self -Wfloat-equal -fPIC)

# Debug Flags
set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG")

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")

# MacOS X needed variables
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # adaptable according to individual needs
  set(CMAKE_MACOSX_RPATH ON)
  #    set(CMAKE_SKIP_BUILD_RPATH FALSE)
  #    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  #    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  #    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif ()

# Finding dependencies
# _____________________________________________________________________________

# Find the Boost libraries and headers.
find_package(Boost COMPONENTS REQUIRED
             "system"
             "unit_test_framework"
             )

# Find and include Gazebo
# LOCAL_GAZEBO_DIR can be set to a path with a gazebo-config.cmake
if (LOCAL_GAZEBO_DIR)
  find_package(gazebo 6.5 REQUIRED CONFIG
               PATHS "${LOCAL_GAZEBO_DIR}"
               NO_DEFAULT_PATH)
  message(WARNING "Using local Gazebo @ ${gazebo_DIR}")
else ()
  find_package(gazebo 6.5 REQUIRED)
endif ()
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_PROTO_INCLUDE})

# Add Gazebo C++ flags (this includes c++11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Need to include the Gazebo protobuf include path
set(GAZEBO_PROTOBUF_DIR)
foreach (ITR ${GAZEBO_INCLUDE_DIRS})
  if (ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_PROTO_PATH "${ITR}/gazebo/msgs/proto")
    set(GAZEBO_PROTO_INCLUDE "${ITR}/gazebo/msgs")
  endif ()
endforeach ()
include_directories(${GAZEBO_PROTO_INCLUDE})

# Find protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

# Find GSL (Gnu mathematical library)
find_package(GSL REQUIRED)

# add revolve-brain
if (NOT DEFINED REVOLVE_BRAIN_PATH)
  message("REVOLVE_BRIAN_PATH is not defined, using default value")
  set(REVOLVE_BRAIN_PATH "../../revolve-brain/cpp")
endif()

add_subdirectory(${REVOLVE_BRAIN_PATH} revolve-brain)
include_directories(
        ${REVOLVE_BRAIN_PATH}
        ${REVOLVE_BRAIN_PATH}/neat/accneat/src
)

if (NOT DEFINED REVOLVE_BUILD_PATH)
  message("REVOLVE_BRIAN_PATH is not defined, using default value")
  set(REVOLVE_BUILD_PATH "../../revolve/build")
  set(REVOLVE_INCLUDE_PATH ${REVOLVE_BUILD_PATH}/include)
  set(REVOLVE_LIBRARIES ${REVOLVE_BUILD_PATH}/lib/librevolve-gazebo.a ${REVOLVE_BUILD_PATH}/lib/librevolve-spec.a)
else ()
  message("Deriving include path / libary path from custom build path: REVOLVE_BUILD_PATH=\"${REVOLVE_BUILD_PATH}\".")
  set(REVOLVE_INCLUDE_PATH ${REVOLVE_BUILD_PATH}/include)
  set(REVOLVE_LIBRARIES ${REVOLVE_BUILD_PATH}/lib/librevolve-gazebo.a ${REVOLVE_BUILD_PATH}/lib/librevolve-spec.a)
endif ()

if (NOT DEFINED REVOLVE_INCLUDE_PATH)
  # Add Revolve as an external project
  # TODO In the future this should fetch from a repository instead
  message("Adding REVOLVE as external project.")
  message("To prevent this, pass REVOLVE_INCLUDE_PATH and REVOLVE_LIBRARIES or REVOLVE_BUILD_PATH.")
  include(ExternalProject)
  externalproject_add(
          revolve
          URL file://${CMAKE_SOURCE_DIR}/../../revolve/cpp
          SOURCE_DIR ${CMAKE_SOURCE_DIR}/../../revolve/cpp
          INSTALL_DIR revolve
          CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/revolve
  )

  # Get Revolve lib / include directories
  externalproject_get_property(revolve INSTALL_DIR)
  set(REVOLVE_INCLUDE_PATH ${INSTALL_DIR}/include)
  set(REVOLVE_LIBRARIES ${INSTALL_DIR}/lib/librevolve-gazebo.a ${INSTALL_DIR}/lib/librevolve-spec.a)
  #    add_dependencies(tolmodelcontrol revolve)
endif ()

# Evolution keys library
find_package(Qt4)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

# Source subdirectories
# _____________________________________________________________________________

# Plugin CPP files
file(GLOB_RECURSE
     TOL_WORLD_SRC
     tol/plugin/WorldController.cpp
     )

file(GLOB_RECURSE
     TOL_ROBOT_SRC
     tol/plugin/brain/Evaluator.cpp
     tol/plugin/RobotController.cpp
     tol/plugin/brain/FakeBrain.cpp
     tol/plugin/brain/Helper.cpp
     tol/plugin/brain/RLPower.cpp
     tol/plugin/brain/RLPower_CPPN.cpp
     tol/plugin/brain/RLPower_Splines.cpp
     tol/plugin/brain/MLMPCPGBrain.cpp
     tol/plugin/brain/Body.cpp
     tol/plugin/brain/YamlBodyParser.cpp
     tol/plugin/brain/NEAT_CPPN.cpp
     tol/plugin/brain/HyperNEAT_CPPN.cpp
     tol/plugin/brain/HyperNEAT_Splines.cpp
     tol/plugin/brain/GenericLearnerBrain.cpp
     tol/plugin/brain/SUPGBrain.cpp
     tol/plugin/brain/SUPGBrainPhototaxis.cpp
     tol/plugin/FakeLightSensor.cpp
     tol/plugin/supg/supgneuron.cpp
     tol/plugin/supg/supggenomemanager.cpp
     )

add_library(tolworldcontrol SHARED ${TOL_WORLD_SRC})
add_library(tolrobotcontrol SHARED ${TOL_ROBOT_SRC})

# Set include dirs and dependencies
include_directories(${Boost_INCLUDE_DIRS}
                    ${REVOLVE_INCLUDE_PATH}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    )
set(TOL_DEPS ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES} ${REVOLVE_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${GSL_LIBRARIES}
    )

# Generate
# _____________________________________________________________________________
# Add the file that registers the body analyzer as a separate library that
# can be used as a world plugin directly.

target_link_libraries(tolrobotcontrol
                      ${TOL_DEPS}
                      revolve-brain-static
                      )
target_link_libraries(tolworldcontrol
                      ${TOL_DEPS}
                      tolrobotcontrol
                      )

# Generate evolution keys library
# TODO: Use CMAKE_AUTOMOC
qt4_wrap_cpp(headers_MOC
             tol/plugin/InteractiveEvolutionPlugin.h
             )
add_library(tolinteractiveevolution SHARED
            ${headers_MOC}
            tol/plugin/InteractiveEvolutionPlugin.cpp
            )
target_link_libraries(tolinteractiveevolution
                      ${GAZEBO_LIBRARIES}
                      ${QT_LIBRARIES}
                      ${PROTOBUF_LIBRARIES}
                      )


add_subdirectory(tol/plugin/test)
