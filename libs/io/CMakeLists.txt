cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

get_filename_component(FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" FOLDER_NAME ${FOLDER_NAME})
project(jpcc_${FOLDER_NAME} VERSION ${JPCC_VERSION})

file(GLOB_RECURSE INCS include/*.h include/*.hpp)
file(GLOB_RECURSE SRCS src/*.cpp)

include_directories(${PCAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} ${INCS} ${SRCS})

add_library(JPCC::${FOLDER_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include JPCC::common JPCC::lvx_common)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb Boost::program_options ${PCL_LIBRARIES} ${PCAP_LIBRARIES} JPCC::common JPCC::lvx_common)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
