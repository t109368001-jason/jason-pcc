JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_include_directories(
        ${LIBRARY_NAME} PRIVATE
        ${TMC2_SOURCE_DIR}/source/lib/PccLibCommon/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibDecoder/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibConformance/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibBitstreamReader/include
        ${TMC2_SOURCE_DIR}/dependencies/program-options-lite
        ${TMC2_SOURCE_DIR}/dependencies/tbb/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibConformance/include
)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        JPCC::common JPCC::decoder_common
)

target_link_libraries(
        ${LIBRARY_NAME} PRIVATE
        PccLibCommon PccLibDecoder PccLibConformance tbb_static PccLibBitstreamCommon PccLibBitstreamReader
)

target_compile_definitions(
        ${LIBRARY_NAME} PUBLIC
        HAVE_MPEG_PCC_TMC2
)
