cmake_minimum_required (VERSION 3.0)
project (FicTrac)

# The version number.
set(FICTRAC_VERSION_MAJOR 2)
set(FICTRAC_VERSION_MIDDLE 1)
set(FICTRAC_VERSION_MINOR 0)
 
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
option(BASLER_USB3 "Use Pylon SDK to capture from Basler USB3 cameras" OFF) # Disabled by default
if(PGR_USB3)
    set(PGR_DIR "." CACHE PATH "Path to PGR Spinnaker SDK folder")
elseif(PGR_USB2)
    set(PGR_DIR "." CACHE PATH "Path to PGR FlyCapture SDK folder")
elseif(BASLER_USB3)
    set(BASLER_DIR "." CACHE PATH "Path to Basler Pylon SDK folder")
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
    elseif(BASLER_USB3)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${BASLER_DIR}/Development/lib/x64)
		find_library(BASLER_LIB PylonBase_MD_VC120_v5_0.lib)
		
		file(GLOB BASLER_LIBS ${BASLER_DIR}/Development/lib/x64/*.lib)
    endif()
    
else()  # gcc
    if(PGR_USB3)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR})
        find_library(PGR_LIB libSpinnaker.so)
    elseif(PGR_USB2)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PGR_DIR})
        find_library(PGR_LIB libflycapture.so)
    elseif(BASLER_USB3)
        set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${BASLER_DIR}/lib64)
        find_library(BASLER_LIB libpylonbase.so)       
        
		file(GLOB BASLER_LIBS ${BASLER_DIR}/lib64/*.so)
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
elseif(BASLER_USB3)
    get_filename_component(BASLER_DIR ${BASLER_LIB} DIRECTORY)
    get_filename_component(BASLER_DIR ${BASLER_DIR} DIRECTORY)    # step up 1 level
    if(MSVC)
        get_filename_component(BASLER_DIR ${BASLER_DIR} DIRECTORY)    # step up 1 level
    endif()
    if(BASLER_LIB)
        message(STATUS "Found Basler Pylon lib ${BASLER_LIB}")
    else()
        message(FATAL_ERROR "Error! Could not find Basler Pylon lib at ${BASLER_DIR}!")
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
elseif(BASLER_USB3)
    include_directories(${BASLER_DIR}/include)
endif()

# find sources to build
file(GLOB LIBFICTRAC_SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)

# add targets
add_library(fictrac_core STATIC ${LIBFICTRAC_SRCS})
add_executable(configGui ${PROJECT_SOURCE_DIR}/exec/configGui.cpp)
add_executable(fictrac ${PROJECT_SOURCE_DIR}/exec/fictrac.cpp)

# add preprocessor definitions
# PUBLIC means defs will be inherited by linked executables
target_compile_definitions(fictrac_core PUBLIC _CRT_SECURE_NO_WARNINGS NOMINMAX)
if(MSVC)
    target_compile_definitions(fictrac_core PUBLIC _WIN32_WINNT=0x0A00)	# Win10
endif()
if(PGR_USB2)
    target_compile_definitions(fictrac_core PUBLIC PGR_USB2)
elseif(PGR_USB3)
    target_compile_definitions(fictrac_core PUBLIC PGR_USB3)
elseif(BASLER_USB3)
    target_compile_definitions(fictrac_core PUBLIC BASLER_USB3)
endif()

# add compile options
if(MSVC)
    target_compile_options(fictrac_core PUBLIC $<$<CONFIG:Release>:/MP /GS /GL /w /WX- /Gy /Zc:wchar_t /O2 /Oi /Zc:inline /fp:precise /MD /EHsc /std:c++17>)
else()  # gcc
    target_compile_options(fictrac_core PUBLIC -Ofast -w -c -fmessage-length=0 -std=c++17 -Wno-unused-function -march=native -MMD)
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
elseif(BASLER_USB3)
    target_link_libraries(fictrac_core PUBLIC ${BASLER_LIBS})
endif()

target_link_libraries(configGui fictrac_core)
add_dependencies(configGui fictrac_core)
target_link_libraries(fictrac fictrac_core)
add_dependencies(fictrac fictrac_core)

if(MSVC)
	set_target_properties(configGui PROPERTIES LINK_FLAGS /LTCG)
	set_target_properties(fictrac PROPERTIES LINK_FLAGS /LTCG)
endif()
