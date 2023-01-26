cmake_minimum_required(VERSION 3.1.0)
project(AdaptiveController) ##EDIT THIS WITH YOUR PROJECT NAME

## S0: Some boilerplate
add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## S1: Finding prebuilt packages
find_package( OpenCV REQUIRED )
find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
find_package(Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

## S2: Finding custom libraries
find_library(MID HCoilMiddlewareLib )
find_library(PSU  DxkdpLib )
find_library(LINACT LinActuatorLib )
find_library(TMETER LakeshoreF71Lib )
find_library(SERIAL CppLinuxSerial )
find_library( ASTAR a-star)

## S3: Set variables for library linkage. Use one of the below only
set(MiddlewareLibs ${MID} ${PSU} ${LINACT} ${TMETER} ${SERIAL} Threads::Threads)
set(PSULibs ${PSU} ${SERIAL})
set(LinActLibs ${LINACT} ${SERIAL})
set(TmeterLibs ${TMETER} ${SERIAL})
set(VisionLibs ${Pylon_LIBRARIES} ${OpenCV_LIBS} ${ASTAR})
set(EIGEN Eigen3::Eigen)
set(THREADS Threads::Threads)

## S4: Set include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/opt/pylon/include)

## S5: Extra Sources
set(CameraFuncs "extraFunctions/extraCameraFuncs.cpp")
set(CompFuncs "extraFunctions/extraCompFuncs.cpp")

add_executable( controllerPrototype "src/ControllerPrototype.cpp")
add_executable( AdaptiveController "src/AdaptiveController.cpp")
add_executable( field3D "src/set3DField.cpp")
add_executable( th_cal "src/thresholds_cal.cpp")
add_executable( run_camera "src/run_camera")

target_link_libraries( controllerPrototype ${EIGEN} ${VisionLibs} ${MiddlewareLibs})
target_link_libraries( AdaptiveController ${EIGEN} ${VisionLibs} ${MiddlewareLibs})
target_link_libraries(field3D ${MiddlewareLibs})
target_link_libraries( th_cal ${VisionLibs})
target_link_libraries( run_camera ${EIGEN} ${VisionLibs})

target_sources(controllerPrototype PRIVATE ${CameraFuncs} ${CompFuncs})
target_sources(AdaptiveController PRIVATE ${CameraFuncs} ${CompFuncs})


