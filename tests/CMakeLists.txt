cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(AllTests VERSION ${JPCC_VERSION})

file(GLOB_RECURSE INCS *.h *.hpp)
file(GLOB_RECURSE SRCS *.cpp)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} ${INCS} ${SRCS} ../libs/decoder/src/decoder.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
        ${PROJECT_NAME}
        PRIVATE
        gtest_main ${PCL_LIBRARIES}
        JPCC::pcc_common JPCC::common JPCC::io JPCC::lvx_common JPCC::math JPCC::octree JPCC::process
        JPCC::visualization
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

include(GoogleTest)

gtest_add_tests(${PROJECT_NAME} "" AUTO)