cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(JPCC_VERSION 0.1)

project(jason-pcc VERSION ${JPCC_VERSION})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if (WIN32)
    macro(get_WIN32_WINNT version)
        if (WIN32 AND CMAKE_SYSTEM_VERSION)
            set(ver ${CMAKE_SYSTEM_VERSION})
            string(REPLACE "." "" ver ${ver})
            string(REGEX REPLACE "([0-9])" "0\\1" ver ${ver})
            set(${version} "0x${ver}")
        endif ()
    endmacro()

    get_WIN32_WINNT(WIN32_WINNT)
    add_definitions(-D_WIN32_WINNT=${WIN32_WINNT})
endif (WIN32)

MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

if (NOT WIN32)
    find_package(TBB REQUIRED)
    if (NOT TBB_FOUND)
        message(FATAL_ERROR "TBB not found")
    endif (NOT TBB_FOUND)
endif (NOT WIN32)

find_package(Boost 1.74 REQUIRED COMPONENTS program_options)

find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)

find_package(PCAP REQUIRED)

find_package(PCL 1.12.0 REQUIRED COMPONENTS common features filters io octree sample_consensus visualization)

find_package(VTK 9.0.3 REQUIRED COMPONENTS CommonCore)

add_subdirectory(3rdparty)

include(cmake/jpcc-target.cmake)

add_subdirectory(libs)

add_subdirectory(app)
