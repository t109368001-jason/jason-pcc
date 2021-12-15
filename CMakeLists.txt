cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(jason-pcc C CXX)

set(CMAKE_CXX_STANDARD            20)
set(CMAKE_CXX_STANDARD_REQUIRED   ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Boost 1.65.0 REQUIRED COMPONENTS system program_options)
add_definitions(-DHAVE_BOOST)
if(WIN32)
macro(get_WIN32_WINNT version)
    if (WIN32 AND CMAKE_SYSTEM_VERSION)
        set(ver ${CMAKE_SYSTEM_VERSION})
        string(REPLACE "." "" ver ${ver})
        string(REGEX REPLACE "([0-9])" "0\\1" ver ${ver})

        set(${version} "0x${ver}")
    endif()
endmacro()

get_WIN32_WINNT(WIN32_WINNT)
add_definitions(-D_WIN32_WINNT=${WIN32_WINNT})
endif(WIN32)

find_package(VTK 7.1 REQUIRED)

find_package(PCL 1.3 REQUIRED COMPONENTS common visualization)

find_package(PCAP REQUIRED)
add_definitions(-DHAVE_PCAP)

add_subdirectory(libs/PCCCommon)
add_subdirectory(libs/JPCCCommon)
add_subdirectory(libs/JPCCIO)

add_subdirectory(app/JPCCTest)
