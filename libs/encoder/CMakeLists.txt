add_subdirectory(encoder_common)
if (${USE_MPEG_PCC_TMC2})
    add_subdirectory(encoder_tmc2)
endif ()
add_subdirectory(encoder_tmc3)

JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        JPCC::common JPCC::encoder_common JPCC::encoder_tmc3
)

if (${USE_MPEG_PCC_TMC2})
    target_link_libraries(${LIBRARY_NAME} PUBLIC JPCC::encoder_tmc2)
endif ()