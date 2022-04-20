JPCC_ADD_LIBRARY(LIBRARY_NAME)

if (VTK_VERSION VERSION_LESS "8.90.0")
    include(${VTK_USE_FILE})
endif ()

target_include_directories(${LIBRARY_NAME} PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_NAME} PUBLIC
        ${PCL_LIBRARIES}
        JPCC::common)
target_link_libraries(${LIBRARY_NAME} PRIVATE Boost::program_options ${VTK_LIBRARIES})

target_compile_definitions(${LIBRARY_NAME} PUBLIC ${PCL_DEFINITIONS})
