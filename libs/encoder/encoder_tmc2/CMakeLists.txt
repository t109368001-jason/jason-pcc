JPCC_ADD_LIBRARY(LIBRARY_NAME)

INCLUDE_DIRECTORIES(
        ${TMC2_SOURCE_DIR}/source/lib/PccLibCommon/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibEncoder/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
        ${TMC2_SOURCE_DIR}/source/lib/PccLibBitstreamWriter/include
        ${TMC2_SOURCE_DIR}/dependencies/program-options-lite
        ${TMC2_SOURCE_DIR}/dependencies/tbb/include
        ${TMC2_SOURCE_DIR}/dependencies/nanoflann
)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        JPCC::common JPCC::encoder_common
)

target_link_libraries(
        ${LIBRARY_NAME} PRIVATE
        PccLibCommon PccLibEncoder tbb_static PccLibBitstreamCommon PccLibBitstreamWriter
)
