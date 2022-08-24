CMAKE_MINIMUM_REQUIRED(VERSION 3.0 FATAL_ERROR)

PROJECT(TMC3 C CXX)
SET(TMC3_VERSION release-v14.0)
SET(TMC3_DIR ${CMAKE_SOURCE_DIR}/3rdparty/mpeg-pcc-tmc13/)
MESSAGE("Clone and build mpeg-pcc-tmc13 libraries: ${TMC3_LIB_SOURCE_DIR}")

IF (NOT EXISTS "${TMC3_DIR}/README.md")
    MESSAGE("mpeg-pcc-tmc13 clone: ${TMC3_DIR}")
    EXECUTE_PROCESS(COMMAND git clone --depth 1 --branch ${TMC3_VERSION} https://github.com/MPEGGroup/mpeg-pcc-tmc13.git ${TMC3_DIR} RESULT_VARIABLE ret)
    IF (NOT ${ret} EQUAL "0")
        MESSAGE(FATAL_ERROR "Error during the mpeg-pcc-tmc13 git clone process. Check that git is well installed on your system.")
    ENDIF ()
ELSE ()
    MESSAGE("mpeg-pcc-tmc13 already cloned: ${TMC3_DIR}")
ENDIF ()

IF (NOT EXISTS "${TMC3_DIR}/PATCHED")
    FOREACH (TMC3_PATCH
            "${CMAKE_SOURCE_DIR}/3rdparty/patch/mpeg-pcc-tmc13-change-path.patch"
            "${CMAKE_SOURCE_DIR}/3rdparty/patch/mpeg-pcc-tmc13-as-library.patch"
            )
        MESSAGE("mpeg-pcc-tmc13 patch: ${TMC3_DIR} (${TMC3_PATCH})")
        EXECUTE_PROCESS(COMMAND git apply ${TMC3_PATCH} --whitespace=nowarn WORKING_DIRECTORY ${TMC3_DIR} RESULT_VARIABLE ret)
        IF (NOT ${ret} EQUAL "0")
            MESSAGE(FATAL_ERROR "Error during the mpeg-pcc-tmc13 patch process. Check that git is well installed on your system.")
        ENDIF ()
    ENDFOREACH ()
    FILE(WRITE ${TMC3_DIR}/PATCHED "mpeg-pcc-tmc13 patched with: " ${TMC3_PATCH})
ELSE ()
    MESSAGE("mpeg-pcc-tmc13 already patched: ${TMC3_DIR}")
ENDIF ()