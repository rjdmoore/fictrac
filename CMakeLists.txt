cmake_minimum_required (VERSION 3.0)
project (FicTrac)

# The version number.
set (FICTRAC_VERSION_MAJOR 2)
set (FICTRAC_VERSION_MINOR 1)
 
# output version info to be included by project
configure_file (
    "${PROJECT_SOURCE_DIR}/include/fictrac_version.h.in"
    "${PROJECT_SOURCE_DIR}/include/fictrac_version.h"
)

# dependency search dirs
set(OPENCV_DIR "." CACHE PATH "Path to OpenCV folder containing OpenCVConfig.cmake")
set(NLOPT_DIR "." CACHE PATH "Path to NLopt folder containing NLoptConfig.cmake")

# output dirs
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# optional build config
option(PGR_USB3 "Use Spinnaker SDK to capture from PGR USB3 cameras" OFF) # Disabled by default
option(PGR_USB2 "Use FlyCapture SDK to capture from PGR USB2 cameras" OFF) # Disabled by default
if(PGR_USB3)
    set(PGR_DIR "." CACHE PATH "Path to PGR Spinnaker SDK folder")
elseif(PGR_USB2)
    set(PGR_DIR "." CACHE PATH "Path to PGR FlyCapture SDK folder")
endif()

# find dependencies
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${OPENCV_DIR} ${NLOPT_DIR})
find_package(OpenCV REQUIRED)
find_package(NLopt CONFIG REQUIRED)
if(NLopt_FOUND)
    get_filename_component(NLopt_INCLUDE_DIRS "${NLOPT_CONFIG_FILE}/../../../include" REALPATH)
    # set(NLopt_LIBS ${NLOPT_LIBRARY_DIRS}/lib/nlopt.lib)
    message(STATUS "Found NLopt: ${NLOPT_CONFIG_FILE}")
else()
    message(FATAL_ERROR "Error! Could not find NLopt lib at ${NLOPT_LIB}!")
endif()
if(MSVC)
    if(PGR_USB3)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR}/lib64/vs2015)
        find_library(PGR_LIB Spinnaker_v140.lib)
    elseif(PGR_USB2)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR}/lib64/vs2015)
        find_library(PGR_LIB FlyCapture2_v140.lib)
    endif()
else()  # gcc
    if(PGR_USB3)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR})
        find_library(PGR_LIB libSpinnaker.so)
    elseif(PGR_USB2)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR})
        find_library(PGR_LIB libflycapture.so)
    endif()
endif()
if(PGR_USB2 OR PGR_USB3)
    get_filename_component(PGR_DIR ${PGR_LIB} DIRECTORY)
    get_filename_component(PGR_DIR ${PGR_DIR} DIRECTORY)    # step up 1 level
    if(MSVC)
        get_filename_component(PGR_DIR ${PGR_DIR} DIRECTORY)    # step up 1 level
    endif()
    if(PGR_LIB)
        message(STATUS "Found PGR FlyCapture/Spinnaker lib ${PGR_LIB}")
    else()
        message(FATAL_ERROR "Error! Could not find PGR FlyCapture/Spinnaker lib at ${PGR_DIR}!")
    endif()
endif()

# add include dirs
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${NLopt_INCLUDE_DIRS})
if(PGR_USB2 OR PGR_USB3)
    if(MSVC)
        include_directories(${PGR_DIR}/include)
    else()
        if(PGR_USB2)
            include_directories(${PGR_DIR}/include/flycapture)   # for ubuntu default install dir
        elseif(PGR_USB3)
            include_directories(${PGR_DIR}/include/spinnaker)   # for ubuntu default install dir
        endif()
    endif()
endif()

# find sources to build
file(GLOB LIBFICTRAC_SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)

# add targets
add_library(fictrac_core STATIC ${LIBFICTRAC_SRCS})
add_executable(configGui ${PROJECT_SOURCE_DIR}/exec/configGui.cpp)
add_executable(fictrac ${PROJECT_SOURCE_DIR}/exec/fictrac.cpp)

# add preprocessor definitions
# public means defs will be inherited by linked executables
target_compile_definitions(fictrac_core PUBLIC _CRT_SECURE_NO_WARNINGS NOMINMAX)
if(PGR_USB2)
    target_compile_definitions(fictrac_core PUBLIC PGR_USB2)
elseif(PGR_USB3)
    target_compile_definitions(fictrac_core PUBLIC PGR_USB3)
endif()

# add compile options
if(MSVC)
    target_compile_options(fictrac_core PUBLIC $<$<CONFIG:Release>:/MP /GS /GL /W3 /WX- /Gy /Zc:wchar_t /O2 /Oi /Zc:inline /fp:precise /MD /EHsc /std:c++17>)
    # set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
else()  # gcc
    target_compile_options(fictrac_core PUBLIC -Ofast -Wall -c -fmessage-length=0 -std=c++17 -Wno-unused-function -march=native -MMD)
endif()

# linking
target_link_libraries(fictrac_core PUBLIC ${OpenCV_LIBS} ${NLOPT_LIBRARIES})
if(MSVC)
    target_link_libraries(fictrac_core PUBLIC Ws2_32)
else()  # gcc
    target_link_libraries(fictrac_core PUBLIC pthread)
endif()
if(PGR_USB2 OR PGR_USB3)
    target_link_libraries(fictrac_core PUBLIC ${PGR_LIB})
endif()

# post-build
if(MSVC)
    # copy all opencv dlls
    set(OPENCV_VER_STRING ${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH})
    foreach(lib ${OpenCV_LIBS})
        if(EXISTS "${OpenCV_INSTALL_PATH}/bin/${lib}${OPENCV_VER_STRING}.dll")
            list(APPEND TO_COPY "${OpenCV_INSTALL_PATH}/bin/${lib}${OPENCV_VER_STRING}.dll")
        endif()
    endforeach()
    
    set(FFMPEG_LIB_BASE opencv_ffmpeg${OPENCV_VER_STRING})
    if("${OpenCV_ARCH}" STREQUAL x86)
        set(FFMPEG_LIB ${FFMPEG_LIB_BASE}.dll)
    else()
        # default to 64-bit
        set(FFMPEG_LIB ${FFMPEG_LIB_BASE}_64.dll)
    endif()
    list(APPEND TO_COPY "${OpenCV_INSTALL_PATH}/bin/${FFMPEG_LIB}")
    
    # copy nlopt dll
    list(APPEND TO_COPY "${NLOPT_LIBRARY_DIRS}/bin/nlopt.dll")
    
    # copy h264 dll
    file(GLOB DLLS ${PROJECT_SOURCE_DIR}/dll/*.dll)
    list(APPEND TO_COPY "${DLLS}")
    
    add_custom_command( TARGET fictrac POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${TO_COPY}
                        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<$<CONFIG:Release>:Release>$<$<CONFIG:Debug>:Debug>/")
else()  # gcc
    # nothing here...
endif()

target_link_libraries(configGui fictrac_core)
add_dependencies(configGui fictrac_core)
target_link_libraries(fictrac fictrac_core)
add_dependencies(fictrac fictrac_core)
