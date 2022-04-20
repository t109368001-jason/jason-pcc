JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_include_directories(${LIBRARY_NAME} PUBLIC ${PCL_LIBRARY_DIRS})

target_link_libraries(${LIBRARY_NAME} PUBLIC
        ${PCL_LIBRARIES} ${PCAP_LIBRARIES}
        JPCC::common JPCC::lvx_common)
target_link_libraries(${LIBRARY_NAME} PRIVATE Boost::program_options)

target_compile_definitions(${LIBRARY_NAME} PUBLIC ${PCL_DEFINITIONS})
