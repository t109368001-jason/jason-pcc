JPCC_ADD_EXECUTABLE(EXECUTABLE_NAME)

target_link_libraries(${EXECUTABLE_NAME} PRIVATE
        Boost::program_options
        JPCC::pcc_common JPCC::common JPCC::io JPCC::metric JPCC::process JPCC::segmentation JPCC::coder)

# TODO remove if unnecessary
target_compile_options(${EXECUTABLE_NAME} PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
        $<$<CXX_COMPILER_ID:GNU>:-Wa,-mbig-obj>)
