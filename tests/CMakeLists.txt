cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(AllTests VERSION ${JPCC_VERSION})

file(GLOB_RECURSE INCS *.h *.hpp)
file(GLOB_RECURSE SRCS *.cpp)

add_executable(${PROJECT_NAME} ${INCS} ${SRCS})

target_link_libraries(${PROJECT_NAME} PRIVATE gtest_main JPCC::octree)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

include(GoogleTest)

gtest_add_tests(${PROJECT_NAME} "" AUTO)