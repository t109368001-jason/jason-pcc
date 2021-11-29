CMAKE_MINIMUM_REQUIRED(VERSION 3.16 FATAL_ERROR)

PROJECT(VelodyneCapture C CXX)
SET(VC_VERSION         VelodyneCapture-27a97e0)
SET(VC_DIR             ${CMAKE_SOURCE_DIR}/3rdparty/${VC_VERSION}/)
set(VC_INCLUDE_DIR     ${VC_DIR}/)
set(VC_LIBRARIES       ${PROJECT_NAME})
MESSAGE("Clone and build VelodyneCapture libraries: ${VC_LIB_SOURCE_DIR}") 

IF(NOT EXISTS "${VC_DIR}/README.md")
  MESSAGE("VelodyneCapture clone: ${VC_DIR}")
  EXECUTE_PROCESS(COMMAND git clone https://github.com/UnaNancyOwen/VelodyneCapture.git ${VC_DIR} RESULT_VARIABLE ret)
  EXECUTE_PROCESS(COMMAND git reset --hard 27a97e0d5150b0111a8914ad701041a3c991460e WORKING_DIRECTORY ${VC_DIR})
  IF(NOT ${ret} EQUAL "0")
    MESSAGE(FATAL_ERROR "Error during the VelodyneCapture git clone process. Check that git is well installed on your system.")
  ENDIF()  
ELSE()
  MESSAGE("VelodyneCapture already cloned: ${VC_DIR}")
ENDIF()

IF(NOT EXISTS "${VC_DIR}/PATCHED")
  SET(VC_PATCH ${CMAKE_SOURCE_DIR}/3rdparty/patch/${VC_VERSION}.patch)
  MESSAGE("VelodyneCapture patch: ${VC_DIR} (${VC_PATCH})")
  EXECUTE_PROCESS(COMMAND git apply ${VC_PATCH} --whitespace=nowarn WORKING_DIRECTORY ${VC_DIR} RESULT_VARIABLE ret)
  IF(NOT ${ret} EQUAL "0")
    MESSAGE(FATAL_ERROR "Error during the VelodyneCapture patch process. Check that git is well installed on your system.")
  ENDIF()
  FILE(WRITE ${VC_DIR}/PATCHED "VelodyneCapture patched with: " ${VC_PATCH})   
ELSE()
  MESSAGE("VelodyneCapture already patched: ${VC_DIR}")
ENDIF()

FILE(GLOB SRC "${VC_INCLUDE_DIR}/*.h" "${VC_INCLUDE_DIR}/*.cpp")

ADD_LIBRARY(${PROJECT_NAME} ${SRC})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} Boost::system ${PCAP_LIBRARIES})

IF(UNIX)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} pthread)
ENDIF(UNIX)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)