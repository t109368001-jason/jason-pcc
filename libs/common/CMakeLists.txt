JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_include_directories(${LIBRARY_NAME} PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        ${PCL_LIBRARIES} Boost::log Boost::program_options
)

target_compile_definitions(${LIBRARY_NAME} PUBLIC ${PCL_DEFINITIONS})
