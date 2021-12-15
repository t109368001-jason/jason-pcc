cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB PROJECT_INCLUDE_FILES include/*.h include/*.hpp)
file(GLOB PROJECT_SRC_FILES     src/*.cpp)

source_group(include FILES ${PROJECT_INCLUDE_FILES})
source_group(src     FILES ${PROJECT_SRC_FILES})

include_directories(include ${BOOST_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${JPCCCOMMON_INCLUDE_DIR})
 
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
                     
add_library(${PROJECT_NAME} ${PROJECT_INCLUDE_FILES} ${PROJECT_SRC_FILES})

set(LIBS Boost::program_options ${PCL_LIBRARIES} ${PCAP_LIBRARIES} JPCCCommon)

target_link_libraries(${PROJECT_NAME} ${LIBS})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set(${PROJECT_NAME_UPPER}_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
