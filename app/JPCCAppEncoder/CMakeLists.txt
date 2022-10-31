JPCC_ADD_EXECUTABLE(EXECUTABLE_NAME)

target_link_libraries(${EXECUTABLE_NAME} PRIVATE
        Boost::program_options
        JPCC::pcc_common JPCC::common JPCC::io JPCC::metric JPCC::process JPCC::segmentation JPCC::combination JPCC::encoder JPCC::decoder)
