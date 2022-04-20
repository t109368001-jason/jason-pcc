function(JPCC_ADD_LIBRARY OUT_NAME)
    get_filename_component(FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    string(REPLACE " " "_" FOLDER_NAME ${FOLDER_NAME})

    set(LIBRARY_NAME "jpcc_${FOLDER_NAME}")
    set(${OUT_NAME} ${LIBRARY_NAME} PARENT_SCOPE)

    file(GLOB_RECURSE INCS include/*.h include/*.hpp)
    file(GLOB_RECURSE SRCS src/*.cpp)

    add_library(${LIBRARY_NAME} ${INCS} ${SRCS})

    add_library(JPCC::${FOLDER_NAME} ALIAS ${LIBRARY_NAME})

    target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_include_directories(${LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

    target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_17)
endfunction()

function(JPCC_ADD_EXECUTABLE OUT_NAME)
    get_filename_component(FOLDER_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
    string(REPLACE " " "_" FOLDER_NAME ${FOLDER_NAME})
    set(FOLDER_NAME ${FOLDER_NAME}${CMAKE_DEBUG_POSTFIX})

    set(EXECUTABLE_NAME ${FOLDER_NAME})
    set(${OUT_NAME} ${EXECUTABLE_NAME} PARENT_SCOPE)

    file(GLOB_RECURSE INCS *.h *.hpp)
    file(GLOB_RECURSE SRCS *.cpp)

    add_executable(${EXECUTABLE_NAME} ${INCS} ${SRCS})

    target_compile_features(${EXECUTABLE_NAME} PUBLIC cxx_std_17)
endfunction()