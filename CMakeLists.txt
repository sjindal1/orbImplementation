cmake_minimum_required(VERSION 3.2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

set(APP_NAME feature_detection)
project(${APP_NAME})

set (CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} "C:\\Program Files (x86)\\flann\\include")
include_directories( ${OpenCV_INCLUDE_DIRS})

set(APP_LIB feature_detection_lib)
add_library(${APP_LIB} orb.cpp fast_9.cpp)
add_executable(${APP_NAME} app.cpp)

target_link_libraries(${APP_NAME} ${OpenCV_LIBS} ${APP_LIB})

message("${OpenCV_LIBS}")
