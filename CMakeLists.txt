cmake_minimum_required (VERSION 3.0)
project (FicTrac)

# The version number.
set (FICTRAC_VERSION_MAJOR 2)
set (FICTRAC_VERSION_MINOR 0)
 
# output version info to be included by project
configure_file (
    "${PROJECT_SOURCE_DIR}/include/fictrac_version.h.in"
    "${PROJECT_SOURCE_DIR}/include/fictrac_version.h"
)

# dependency search dirs
SET(OPENCV_DIR "." CACHE PATH "Path to OpenCV folder containing OpenCVConfig.cmake")
SET(NLOPT_DIR "." CACHE PATH "Path to NLOpt folder containing libnlopt-0.lib")

# output dirs
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# find dependencies
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${OPENCV_DIR} ${NLOPT_DIR})
find_package(opencv)
find_library(NLOPT_LIB libnlopt-0.lib)
if(NLOPT_LIB)
    message(STATUS "Found NLOpt lib ${NLOPT_LIB}")
    message(STATUS "You might need to add ${NLOPT_DIR} to your PATH to be able to run the executable.")
else()
    message(WARNING "Warning! Could not find NLOpt lib at ${NLOPT_DIR}")
endif()

# add include dirs
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${NLOPT_DIR})

# find sources to build
file(GLOB LIBFICTRAC_SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)

# add targets
add_library(libfictrac STATIC ${LIBFICTRAC_SRCS})
add_executable(configGui ${PROJECT_SOURCE_DIR}/exec/configGui.cpp)
add_executable(fictrac ${PROJECT_SOURCE_DIR}/exec/fictrac.cpp)

# add preprocessor definitions
# public means defs will be inherited by linked executables
target_compile_definitions(libfictrac PUBLIC _CRT_SECURE_NO_WARNINGS)

# add compile options
if(MSVC)
    target_compile_options(libfictrac PUBLIC $<$<CONFIG:Release>:/MP /GS /GL /W3 /WX- /Gy /Zc:wchar_t /O2 /Oi /Zc:inline /fp:precise /MD /EHsc>)
else()  # gcc
    add_compile_options($<$<CONFIG:Release>:-O3 -Wall -c -fmessage-length=0 -Wno-unused-function -MMD>)
endif()

# linking and post-build
target_link_libraries(libfictrac PUBLIC ${OpenCV_LIBS} ${NLOPT_LIB})
if(MSVC)
    # win-specific libraries
    target_link_libraries(libfictrac PUBLIC Ws2_32)
    
    # copy all opencv dlls
    set(OPENCV_VER_STRING ${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH})
    foreach(lib ${OpenCV_LIBS})
        if(EXISTS "${_OpenCV_LIB_PATH}/${lib}${OPENCV_VER_STRING}.dll")
            list(APPEND TO_COPY "${_OpenCV_LIB_PATH}/${lib}${OPENCV_VER_STRING}.dll" "${_OpenCV_LIB_PATH}/${lib}${OPENCV_VER_STRING}d.dll")
        endif()
    endforeach()
    
    set(FFMPEG_LIB_BASE opencv_ffmpeg${OPENCV_VER_STRING})
    if(${OpenCV_ARCH} STREQUAL x64)
        set(FFMPEG_LIB ${FFMPEG_LIB_BASE}_64.dll)
    else()
        set(FFMPEG_LIB ${FFMPEG_LIB_BASE}.dll)
    endif()
    list(APPEND TO_COPY "${_OpenCV_LIB_PATH}/${FFMPEG_LIB}")
    
    # copy nlopt dll
    list(APPEND TO_COPY "${NLOPT_DIR}/libnlopt-0.dll")
    
    add_custom_command( TARGET fictrac POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${TO_COPY}
                        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<$<CONFIG:Release>:Release>$<$<CONFIG:Debug>:Debug>/")
else()  # gcc
    # socket libs, pthread
endif()

target_link_libraries(configGui libfictrac)
target_link_libraries(fictrac libfictrac)
