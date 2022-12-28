add_subdirectory(decoder_common)
add_subdirectory(decoder_tmc3)

JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        JPCC::common JPCC::decoder_common JPCC::decoder_tmc3
)
