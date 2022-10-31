JPCC_ADD_LIBRARY(LIBRARY_NAME)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        VTK::CommonCore JPCC::common
)

vtk_module_autoinit(
        TARGETS ${LIBRARY_NAME}
        MODULES VTK::CommonCore
)
