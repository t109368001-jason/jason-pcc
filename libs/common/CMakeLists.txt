JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_include_directories(${LIBRARY_NAME} PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        ${PCL_LIBRARIES} Boost::program_options JPCC::pcc_common
)

target_compile_definitions(${LIBRARY_NAME} PUBLIC ${PCL_DEFINITIONS})
