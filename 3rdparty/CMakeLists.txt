
include(cmake/mpeg-pcc-tmc13.cmake)
include(cmake/mpeg-pcc-tmc2.cmake)

add_subdirectory(mpeg-pcc-tmc13)
add_subdirectory(mpeg-pcc-tmc2)

target_include_directories(tmc3 PUBLIC mpeg-pcc-tmc13)
if (MSVC)
    target_compile_options(tmc3 PRIVATE
            /wd4005 /wd4101 /wd4715 /wd4804 /wd4805)
    target_compile_options(tmc3 PUBLIC /wd4805)
else ()
    target_compile_options(tmc3 PRIVATE
            -Wno-return-type -Wno-attributes)
endif ()

