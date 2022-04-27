JPCC_ADD_EXECUTABLE(EXECUTABLE_NAME)

target_link_libraries(${EXECUTABLE_NAME} PRIVATE
        Boost::program_options
        JPCC::pcc_common JPCC::common JPCC::io JPCC::process JPCC::visualization)

vtk_module_autoinit(
        TARGETS ${EXECUTABLE_NAME}
        MODULES ${VTK_LIBRARIES}
)

if (NOT WIN32)
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE pthread)
endif (NOT WIN32)
