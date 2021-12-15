cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set(MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX})
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp *.c)

include_directories(${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${PCCCOMMON_INCLUDE_DIR} ${JPCCCOMMON_INCLUDE_DIR} ${JPCCIO_INCLUDE_DIR})

link_directories(${VTK_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
                     
add_executable(${PROJECT_NAME} ${SRC})

set(LIBS ${PCL_LIBRARIES} PCCCommon JPCCCommon JPCCIO)

target_link_libraries(${PROJECT_NAME} ${LIBS})

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} pthread)
endif(NOT WIN32)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)