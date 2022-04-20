JPCC_ADD_EXECUTABLE(EXECUTABLE_NAME)

target_include_directories(${EXECUTABLE_NAME} PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(${EXECUTABLE_NAME} PRIVATE
        Boost::program_options ${PCL_LIBRARIES}
        JPCC::pcc_common JPCC::common JPCC::io JPCC::octree JPCC::process)
if (NOT WIN32)
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE pthread)
endif (NOT WIN32)

target_compile_definitions(${EXECUTABLE_NAME} PRIVATE ${PCL_DEFINITIONS})