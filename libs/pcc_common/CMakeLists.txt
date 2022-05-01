JPCC_ADD_LIBRARY(LIBRARY_NAME)

if (NOT WIN32)
    include(CheckSymbolExists)
    check_symbol_exists(getrusage sys/resource.h HAVE_GETRUSAGE)

    if (NOT HAVE_GETRUSAGE)
        message(FATAL_ERROR "getrusage not found")
    endif (NOT HAVE_GETRUSAGE)
    target_compile_definitions(${LIBRARY_NAME} PUBLIC -DHAVE_GETRUSAGE)

    target_link_libraries(${LIBRARY_NAME} PUBLIC TBB::tbb)
endif (NOT WIN32)
