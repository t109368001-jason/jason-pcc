JPCC_ADD_EXECUTABLE(EXECUTABLE_NAME)

if(CMAKE_COMPILER_IS_GNUCXX)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-Wa,-mbig-obj" GNU_BIG_OBJ_FLAG_ENABLE)
    message(STATUS GNU_BIG_OBJ_FLAG_ENABLE=${GNU_BIG_OBJ_FLAG_ENABLE})
endif()

target_link_libraries(${EXECUTABLE_NAME} PRIVATE
        Boost::program_options
        JPCC::pcc_common JPCC::common JPCC::io JPCC::metric JPCC::process JPCC::segmentation JPCC::coder)

# TODO remove if unnecessary
target_compile_options(${EXECUTABLE_NAME} PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
        $<$<AND:$<CXX_COMPILER_ID:GNU>,$<BOOL:${GNU_BIG_OBJ_FLAG_ENABLE}>>:-Wa,-mbig-obj>)
