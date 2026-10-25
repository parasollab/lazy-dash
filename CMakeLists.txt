cmake_minimum_required(VERSION 3.20)
project(lazydash VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# The Python module links the static libraries (lazydash, CoMotion, OMPL).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(LAZYDASH_BUILD_TESTS "Build lazydash tests" ON)
set(LAZYDASH_COMOTION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/comotion"
    CACHE PATH "CoMotion checkout (see external/bootstrap_comotion.sh)")

if(NOT EXISTS "${LAZYDASH_COMOTION_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "CoMotion not found at ${LAZYDASH_COMOTION_DIR}; run external/bootstrap_comotion.sh")
endif()

# CoMotion is the backbone; build only its library.
set(COMOTION_BUILD_APPS OFF CACHE BOOL "" FORCE)
set(COMOTION_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(COMOTION_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${LAZYDASH_COMOTION_DIR}" "${CMAKE_BINARY_DIR}/comotion" EXCLUDE_FROM_ALL)

# nlohmann/json: a package found by CoMotion is only visible there, so look it up here too.
if(NOT TARGET nlohmann_json::nlohmann_json)
  find_package(nlohmann_json 3.11 REQUIRED)
endif()

add_library(lazydash
  src/lazydash/planning/LazyDash.cpp
  src/lazydash/planning/ResultJson.cpp
  src/lazydash/robot/Ur5eModel.cpp
  src/lazydash/robot/BodyModels.cpp
  src/lazydash/robot/Formation.cpp
  src/lazydash/motion/LazyRoadmap.cpp
  src/lazydash/motion/MotionLayer.cpp
  src/lazydash/robot/GraspSet.cpp
  src/lazydash/robot/UrIk.cpp
  src/lazydash/scene/Scene.cpp
  src/lazydash/task/Capabilities.cpp
  src/lazydash/task/DurationModel.cpp
  src/lazydash/task/Heuristic.cpp
  src/lazydash/task/TaskConflictDetection.cpp
  src/lazydash/task/TaskConstraints.cpp
  src/lazydash/task/TaskHypergraph.cpp
  src/lazydash/task/TaskSearch.cpp
  src/lazydash/task/TransitionSampler.cpp
  src/lazydash/schedule/MeshVerifier.cpp
  src/lazydash/schedule/MotionConflicts.cpp
  src/lazydash/schedule/ScheduledARC.cpp
  src/lazydash/schedule/TimedSchedule.cpp
)
add_library(lazydash::lazydash ALIAS lazydash)
target_include_directories(lazydash PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(lazydash PUBLIC comotion::comotion PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(lazydash PUBLIC
  LAZYDASH_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")
target_compile_options(lazydash PRIVATE -Wall -Wextra)

add_executable(lazydash_solve apps/lazydash_solve.cpp)
target_link_libraries(lazydash_solve PRIVATE lazydash)

add_executable(ht_diag apps/ht_diag.cpp)
target_link_libraries(ht_diag PRIVATE lazydash)

# Python module lazydash._core (nanobind, vendored by CoMotion's como-ompl).
option(LAZYDASH_BUILD_PYTHON "Build the lazydash Python module" ON)
set(LAZYDASH_NANOBIND_DIR "${LAZYDASH_COMOTION_DIR}/external/como-ompl/external/nanobind"
    CACHE PATH "nanobind source tree")
if(LAZYDASH_BUILD_PYTHON)
  find_package(Python 3.10 COMPONENTS Interpreter Development.Module QUIET)
  if(Python_FOUND AND EXISTS "${LAZYDASH_NANOBIND_DIR}/CMakeLists.txt")
    add_subdirectory("${LAZYDASH_NANOBIND_DIR}" "${CMAKE_BINARY_DIR}/nanobind" EXCLUDE_FROM_ALL)
    nanobind_add_module(lazydash_core python/src/module.cpp)
    target_link_libraries(lazydash_core PRIVATE lazydash)
    set_target_properties(lazydash_core PROPERTIES OUTPUT_NAME _core
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python/lazydash")
  else()
    message(STATUS "lazydash: Python module disabled (Python or nanobind not found)")
  endif()
endif()

if(LAZYDASH_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.1)
  FetchContent_MakeAvailable(Catch2)
  enable_testing()
  add_subdirectory(tests)
endif()
