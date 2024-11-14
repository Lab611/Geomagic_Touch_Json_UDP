cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
#if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
#    add_definitions(-DRELEASE)
#endif ()

project(NE30_controller)

set(EXECUTABLE_NAME main)

set(Eigen3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/eigen-3.4.0")
set(OpenHaptics_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/openhaptics/include")
set(OpenHaptics_LIB_DIR "${PROJECT_SOURCE_DIR}/3rdparty/openhaptics/lib/x64")

include_directories(
        include
        3rdparty
        ${Eigen3_INCLUDE_DIR}
        ${OpenHaptics_INCLUDE_DIR}
)

#AUX_SOURCE_DIRECTORY(src INCLUDE_SRC)
## AUX_SOURCE_DIRECTORY(./ MAIN_SRC)
#add_library(INCLUDE ${INCLUDE_SRC})

add_executable(${EXECUTABLE_NAME} main.cpp)

target_link_libraries(${EXECUTABLE_NAME}
#        INCLUDE
        Ws2_32
)

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    message(STATUS "Release mode")
    message(STATUS "${OpenHaptics_LIB_DIR}/Release/...")
    target_link_libraries(${EXECUTABLE_NAME}
            ${OpenHaptics_LIB_DIR}/Release/hd.lib
            ${OpenHaptics_LIB_DIR}/Release/hl.lib
            ${OpenHaptics_LIB_DIR}/Release/hdu.lib
            ${OpenHaptics_LIB_DIR}/Release/hlu.lib
            ${OpenHaptics_LIB_DIR}/Release/glui32.lib
            ${OpenHaptics_LIB_DIR}/Release/glut32.lib
            ${OpenHaptics_LIB_DIR}/Release/HapticMouse.lib
    )
else ()
    message(STATUS "Debug mode")
    message(STATUS "${OpenHaptics_LIB_DIR}/Debug/...")
    target_link_libraries(${EXECUTABLE_NAME}
            ${OpenHaptics_LIB_DIR}/Debug/hd.lib
            ${OpenHaptics_LIB_DIR}/Debug/hl.lib
            ${OpenHaptics_LIB_DIR}/Debug/hdu.lib
            ${OpenHaptics_LIB_DIR}/Debug/hlu.lib
            ${OpenHaptics_LIB_DIR}/Debug/glui32.lib
            ${OpenHaptics_LIB_DIR}/Debug/glut32.lib
            ${OpenHaptics_LIB_DIR}/Debug/HapticMouse.lib

    )
endif ()

# 使用生成器表达式获取可执行文件的完整路径
set(TARGET_FILE_DIR $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)

## 打印可执行文件的路径（这将在构建时发生）
#add_custom_command(
#        TARGET ${EXECUTABLE_NAME} POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E echo "Executable path: ${TARGET_FILE_DIR}"
#        COMMAND ${CMAKE_COMMAND} -E echo "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}"
#        COMMAND ${CMAKE_COMMAND} -E
#        copy_if_different
#        "$<$<CONFIG:Release>:${INNFOS_LIB_DIR}/Release/hd.dll>"
#        "$<$<CONFIG:Debug>:${INNFOS_LIB_DIR}/Debug/hd.dll>"
#        "${TARGET_FILE_DIR}"
#        COMMAND ${CMAKE_COMMAND} -E
#        copy_if_different
#        "$<$<CONFIG:Release>:${INNFOS_LIB_DIR}/Release/hl.dll>"
#        "$<$<CONFIG:Debug>:${INNFOS_LIB_DIR}/Debug/hl.dll>"
#        "${TARGET_FILE_DIR}"
#)