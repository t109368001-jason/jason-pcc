cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

get_filename_component(FOLDER_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" FOLDER_NAME ${FOLDER_NAME})
set(FOLDER_NAME ${FOLDER_NAME}${CMAKE_DEBUG_POSTFIX})
project(${FOLDER_NAME} VERSION ${JPCC_VERSION})

file(GLOB_RECURSE INCS *.h *.hpp)
file(GLOB_RECURSE SRCS *.cpp)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

if (VTK_VERSION VERSION_LESS "8.90.0")
    include(${VTK_USE_FILE})
endif ()

add_executable(${PROJECT_NAME} ${INCS} ${SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES} ${VTK_LIBRARIES}
        JPCC::pcc_common JPCC::common JPCC::lvx_common JPCC::io JPCC::process JPCC::visualization)
#vtk_module_autoinit(
#        TARGETS ${PROJECT_NAME}
#        MODULES ${VTK_LIBRARIES}
#)

if (NOT WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
endif (NOT WIN32)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
