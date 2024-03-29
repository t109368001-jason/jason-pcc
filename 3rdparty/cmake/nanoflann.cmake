CMAKE_MINIMUM_REQUIRED(VERSION 3.0 FATAL_ERROR)

PROJECT(NANOFLANN C CXX)
SET(NANOFLANN_VERSION v1.4.3)
SET(NANOFLANN_DIR ${CMAKE_SOURCE_DIR}/3rdparty/nanoflann/)
MESSAGE("Clone and build nanoflann libraries: ${NANOFLANN_LIB_SOURCE_DIR}")

IF (NOT EXISTS "${NANOFLANN_DIR}/README.md")
    MESSAGE("nanoflann clone: ${NANOFLANN_DIR}")
    EXECUTE_PROCESS(COMMAND git clone --depth 1 --branch ${NANOFLANN_VERSION} https://github.com/jlblancoc/nanoflann.git ${NANOFLANN_DIR} RESULT_VARIABLE ret)
    IF (NOT ${ret} EQUAL "0")
        MESSAGE(FATAL_ERROR "Error during the nanoflann git clone process. Check that git is well installed on your system.")
    ENDIF ()
ELSE ()
    MESSAGE("nanoflann already cloned: ${NANOFLANN_DIR}")
ENDIF ()

IF (NOT EXISTS "${NANOFLANN_DIR}/PATCHED")
    FOREACH (NANOFLANN_PATCH
            "${CMAKE_SOURCE_DIR}/3rdparty/patch/nanoflann-kdtree-vector-adaptor.patch"
            )
        MESSAGE("nanoflann patch: ${NANOFLANN_DIR} (${NANOFLANN_PATCH})")
        EXECUTE_PROCESS(COMMAND git apply ${NANOFLANN_PATCH} --whitespace=nowarn WORKING_DIRECTORY ${NANOFLANN_DIR} RESULT_VARIABLE ret)
        IF (NOT ${ret} EQUAL "0")
            MESSAGE(FATAL_ERROR "Error during the nanoflann patch process. Check that git is well installed on your system.")
        ENDIF ()
    ENDFOREACH ()
    FILE(WRITE ${NANOFLANN_DIR}/PATCHED "nanoflann patched with: " ${NANOFLANN_PATCH})
ELSE ()
    MESSAGE("nanoflann already patched: ${NANOFLANN_DIR}")
ENDIF ()
