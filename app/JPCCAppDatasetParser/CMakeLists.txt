JPCC_ADD_EXECUTABLE(EXECUTABLE_NAME)

target_link_libraries(
        ${EXECUTABLE_NAME} PRIVATE
        JPCC::common JPCC::io JPCC::process
)
