add_subdirectory(encoder_common)
add_subdirectory(encoder_tmc3)

JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        JPCC::common JPCC::encoder_common JPCC::encoder_tmc3
)
