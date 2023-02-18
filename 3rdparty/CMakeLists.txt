
include(cmake/nanoflann.cmake)
include(cmake/mpeg-pcc-tmc13.cmake)
if (${USE_MPEG_PCC_TMC2})
    include(cmake/mpeg-pcc-tmc2.cmake)
endif ()

set(NANOFLANN_BUILD_EXAMPLES OFF)
set(NANOFLANN_BUILD_BENCHMARKS OFF)
set(NANOFLANN_BUILD_TESTS OFF)

add_subdirectory(nanoflann)
add_subdirectory(mpeg-pcc-tmc13)
if (${USE_MPEG_PCC_TMC2})
    add_subdirectory(mpeg-pcc-tmc2)
endif ()

target_include_directories(tmc3 PUBLIC mpeg-pcc-tmc13)
if (MSVC)
    target_compile_options(tmc3 PRIVATE
            /wd4005 /wd4101 /wd4715 /wd4804 /wd4805)
    target_compile_options(tmc3 PUBLIC /wd4805)
else ()
    target_compile_options(tmc3 PRIVATE
            -Wno-return-type -Wno-attributes)
endif ()
