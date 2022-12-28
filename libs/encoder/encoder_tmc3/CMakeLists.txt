JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        JPCC::common JPCC::encoder_common
)

target_link_libraries(
        ${LIBRARY_NAME} PRIVATE
        tmc3
)
