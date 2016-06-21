cmake_minimum_required(VERSION 3.1)

PROJECT( accneat )

SET(CMAKE_CXX_STANDARD 11)

FIND_PACKAGE(OpenMP)

#TODO enable cuda

SET(accneat_sources
    src/experiments/experiment.cpp
    src/experiments/maze/maze.cpp
    src/experiments/static/cfg.cpp
    src/experiments/static/regex.cpp
    src/experiments/static/sequence.cpp
    src/experiments/static/xor.cpp
    src/genomemanager.cpp
    src/innovgenome/innovation.cpp
    src/innovgenome/innovgenome.cpp
    src/innovgenome/innovgenomemanager.cpp
    src/innovgenome/innovlinkgene.cpp
    src/innovgenome/innovnodegene.cpp
    src/innovgenome/trait.cpp
#     src/main.cpp
    src/neat.cpp
    src/network/cpu/cpunetwork.cpp
    src/organism.cpp
    src/population.cpp
    src/species/species.cpp
    src/species/speciesorganism.cpp
    src/species/speciespopulation.cpp
    src/util/map.cpp
    src/util/resource.cpp
    src/util/rng.cpp
    src/util/timer.cpp
    src/util/util.cpp
)

SET(accneat_cxx_sources
    src/experiments/static/staticevaluator.cxx
    src/experiments/maze/mazeevaluator.cxx
)

INCLUDE_DIRECTORIES( "src" )

add_library( accneat ${accneat_sources} ${accneat_cxx_sources})
target_link_libraries( accneat "gomp")
SET_TARGET_PROPERTIES( accneat PROPERTIES
  COMPILE_FLAGS "${OpenMP_CXX_FLAGS}"
)


