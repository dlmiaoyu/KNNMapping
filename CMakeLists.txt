cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(KNNMapping)

find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

include_directories(${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${YAMLCPP_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE HDR_FILES include/*.hpp)

add_library(KNNMapping SHARED ${SRC_FILES})

set(LIBS
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARIES})

target_link_libraries(KNNMapping
        ${LIBS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(ParameterList test/ParameterList.cpp)
target_link_libraries(ParameterList
        KNNMapping)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(TimeStamp test/TimeStamp.cpp)
target_link_libraries(TimeStamp
        KNNMapping)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(PointCloud test/PointCloud.cpp)
target_link_libraries(PointCloud
        KNNMapping)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(Preprocessing test/Preprocessing.cpp)
target_link_libraries(Preprocessing
        KNNMapping)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(Mapping test/Mapping.cpp)
target_link_libraries(Mapping
        KNNMapping)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(ExtractPose test/ExtractPose.cpp)
target_link_libraries(ExtractPose
        KNNMapping)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(PointNumber test/PointNumber.cpp)
target_link_libraries(PointNumber
        KNNMapping)