cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

get_filename_component(FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" FOLDER_NAME ${FOLDER_NAME})
project(jpcc_${FOLDER_NAME} VERSION ${JPCC_VERSION})

file(GLOB_RECURSE INCS include/*.h include/*.hpp)
file(GLOB_RECURSE SRCS src/*.cpp)

if (NOT WIN32)
    include(CheckSymbolExists)
    check_symbol_exists(getrusage sys/resource.h HAVE_GETRUSAGE)

    if (NOT HAVE_GETRUSAGE)
        message(FATAL_ERROR "getrusage not found")
    endif (NOT HAVE_GETRUSAGE)
    add_definitions(-DHAVE_GETRUSAGE)
    find_package(TBB REQUIRED)
    if (NOT TBB_FOUND)
        message(FATAL_ERROR "TBB not found")
    endif (NOT TBB_FOUND)
endif (NOT WIN32)

add_library(${PROJECT_NAME} ${INCS} ${SRCS})

add_library(JPCC::${FOLDER_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

if (NOT WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
endif (NOT WIN32)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
