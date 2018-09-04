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
set(OPENCV_DIR "." CACHE PATH "Path to OpenCV folder containing OpenCVConfig.cmake")
set(NLOPT_DIR "." CACHE PATH "Path to NLOpt folder containing libnlopt-0.lib")

# output dirs
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# optional build config
option(PGR_USB3 "Use Spinnaker SDK to capture from PGR USB3 cameras" OFF) # Disabled by default
if(PGR_USB3)
    set(PGR_DIR "." CACHE PATH "Path to PGR Spinnaker SDK folder")
endif()

# find dependencies
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${OPENCV_DIR} ${NLOPT_DIR})
if(MSVC)
    find_package(opencv)
    find_library(NLOPT_LIB libnlopt-0.lib)
    
    if(PGR_USB3)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR}/lib64/vs2015)
        find_library(PGR_LIB Spinnaker_v140.lib)
    endif()
else()  # gcc
    find_package(OpenCV)
    find_library(NLOPT_LIB libnlopt.a)
    
    if(PGR_USB3)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR}/lib64) # fixme!
        find_library(PGR_LIB libspinnaker.a) # fixme!
    endif()
endif()
get_filename_component(NLOPT_DIR ${NLOPT_LIB} DIRECTORY)
if(NLOPT_LIB)
    message(STATUS "Found NLOpt lib ${NLOPT_LIB}")
    if(MSVC)
        message(STATUS "You might need to add ${NLOPT_DIR} to your PATH to be able to run the executable.")
    endif()
else()
    message(FATAL_ERROR "Error! Could not find NLOpt lib at ${NLOPT_DIR}!")
endif()
if(PGR_USB3)
    if(PGR_LIB)
        message(STATUS "Found PGR Spinnaker lib ${PGR_LIB}")
    else()
        message(FATAL_ERROR "Error! Could not find PGR Spinnaker lib at ${PGR_DIR}!")
    endif()
endif()

# add include dirs
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${NLOPT_DIR})
if(PGR_USB3)
    include_directories(${PGR_DIR}/include)
endif()

# find sources to build
file(GLOB LIBFICTRAC_SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)

# add targets
add_library(libfictrac STATIC ${LIBFICTRAC_SRCS})
add_executable(configGui ${PROJECT_SOURCE_DIR}/exec/configGui.cpp)
add_executable(fictrac ${PROJECT_SOURCE_DIR}/exec/fictrac.cpp)

# add preprocessor definitions
# public means defs will be inherited by linked executables
target_compile_definitions(libfictrac PUBLIC _CRT_SECURE_NO_WARNINGS NOMINMAX)
if(PGR_USB3)
    target_compile_definitions(libfictrac PUBLIC PGR_USB3)
endif()

# add compile options
if(MSVC)
    target_compile_options(libfictrac PUBLIC $<$<CONFIG:Release>:/MP /GS /GL /W3 /WX- /Gy /Zc:wchar_t /O2 /Oi /Zc:inline /fp:precise /MD /EHsc>)
else()  # gcc
    target_compile_options(libfictrac PUBLIC -Ofast -Wall -c -fmessage-length=0 -std=c++14 -Wno-unused-function -march=native -MMD)
endif()

# linking and post-build
target_link_libraries(libfictrac PUBLIC ${OpenCV_LIBS} ${NLOPT_LIB})
if(PGR_USB3)
    target_link_libraries(libfictrac PUBLIC ${PGR_LIB})
endif()
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
    
    # copy h264 dll
    list(APPEND TO_COPY "${PROJECT_SOURCE_DIR}/dll/openh264-1.7.0-win64.dll")
    
    add_custom_command( TARGET fictrac POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${TO_COPY}
                        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<$<CONFIG:Release>:Release>$<$<CONFIG:Debug>:Debug>/")
else()  # gcc
    # socket libs, pthread
    target_link_libraries(libfictrac PUBLIC pthread)
endif()

target_link_libraries(configGui libfictrac)
add_dependencies(configGui libfictrac)
target_link_libraries(fictrac libfictrac)
add_dependencies(fictrac libfictrac)
