JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        ${PCAP_LIBRARIES} JPCC::lvx_common JPCC::common
)
