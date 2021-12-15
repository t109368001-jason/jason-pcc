cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

if(NOT WIN32)
    include(CheckSymbolExists)
    check_symbol_exists(getrusage sys/resource.h HAVE_GETRUSAGE)
    
    if(NOT HAVE_GETRUSAGE)
        message(FATAL_ERROR "getrusage not found")
    endif(NOT HAVE_GETRUSAGE)
    add_definitions(-DHAVE_GETRUSAGE)
    find_package(TBB REQUIRED)
    if(NOT TBB_FOUND)
        message(FATAL_ERROR "TBB not found")
    endif(NOT TBB_FOUND)
endif(NOT WIN32)

file(GLOB PROJECT_INCLUDE_FILES include/*.h include/*.hpp)
file(GLOB PROJECT_SRC_FILES     src/*.cpp)

include_directories(include)

if(NOT WIN32)
    include_directories(${TBB_INCLUDE_DIRS})
endif(NOT WIN32)
 
add_library(${PROJECT_NAME} ${PROJECT_INCLUDE_FILES} ${PROJECT_SRC_FILES})

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} TBB::tbb)
endif(NOT WIN32)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set(${PROJECT_NAME_UPPER}_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
