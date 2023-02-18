add_subdirectory(decoder_common)
if (${USE_MPEG_PCC_TMC2})
    add_subdirectory(decoder_tmc2)
endif ()
add_subdirectory(decoder_tmc3)

JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        JPCC::common JPCC::decoder_common JPCC::decoder_tmc3
)

if (${USE_MPEG_PCC_TMC2})
    target_link_libraries(${LIBRARY_NAME} PUBLIC JPCC::decoder_tmc2)
endif ()