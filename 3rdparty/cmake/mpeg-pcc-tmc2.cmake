CMAKE_MINIMUM_REQUIRED(VERSION 3.0 FATAL_ERROR)

PROJECT(TMC2 C CXX)
SET(TMC2_VERSION release-v18.0)
SET(TMC2_DIR ${CMAKE_SOURCE_DIR}/3rdparty/mpeg-pcc-tmc2/)
MESSAGE("Clone and build mpeg-pcc-tmc2 libraries: ${TMC2_LIB_SOURCE_DIR}")

IF (NOT EXISTS "${TMC2_DIR}/README.md")
    MESSAGE("mpeg-pcc-tmc2 clone: ${TMC2_DIR}")
    EXECUTE_PROCESS(COMMAND git clone --depth 1 --branch ${TMC2_VERSION} https://github.com/MPEGGroup/mpeg-pcc-tmc2.git ${TMC2_DIR} RESULT_VARIABLE ret)
    IF (NOT ${ret} EQUAL "0")
        MESSAGE(FATAL_ERROR "Error during the mpeg-pcc-tmc2 git clone process. Check that git is well installed on your system.")
    ENDIF ()
ELSE ()
    MESSAGE("mpeg-pcc-tmc2 already cloned: ${TMC2_DIR}")
ENDIF ()

IF (NOT EXISTS "${TMC2_DIR}/PATCHED")
    FOREACH (TMC2_PATCH
            "${CMAKE_SOURCE_DIR}/3rdparty/patch/mpeg-pcc-tmc2-for-jpcc.patch"
            "${CMAKE_SOURCE_DIR}/3rdparty/patch/mpeg-pcc-tmc2-fix-reflectance.patch.patch"
            "${CMAKE_SOURCE_DIR}/3rdparty/patch/mpeg-pcc-tmc2-move-reflectivity-to-color.patch.patch"
            )
        MESSAGE("mpeg-pcc-tmc2 patch: ${TMC2_DIR} (${TMC2_PATCH})")
        EXECUTE_PROCESS(COMMAND git apply ${TMC2_PATCH} --whitespace=nowarn --ignore-whitespace WORKING_DIRECTORY ${TMC2_DIR} RESULT_VARIABLE ret)
        IF (NOT ${ret} EQUAL "0")
            MESSAGE(FATAL_ERROR "Error during the mpeg-pcc-tmc2 patch process. Check that git is well installed on your system.")
        ENDIF ()
    ENDFOREACH ()
    FILE(WRITE ${TMC2_DIR}/PATCHED "mpeg-pcc-tmc2 patched with: " ${TMC2_PATCH})
ELSE ()
    MESSAGE("mpeg-pcc-tmc2 already patched: ${TMC2_DIR}")
ENDIF ()
