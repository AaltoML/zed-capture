cmake_minimum_required(VERSION 3.0)
project(zed_capture)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -O2 -lstdc++")
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/zed-open-capture/cmake/" ${CMAKE_MODULE_PATH})
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")



option (JSON_BuildTests "Build nlohmann_json unit tests" ON)
set(JSON_BuildTests OFF)
set(USE_OPENCV_VIDEO_RECORDING ON CACHE INTERNAL "")
add_definitions(-DUSE_OPENCV_VIDEO_RECORDING)
add_subdirectory("jsonl-recorder")

find_package(OpenCV 4 REQUIRED)
find_package(LIBUSB 1 REQUIRED)
find_package(HIDAPI REQUIRED)
# include_directories(${LibUSB_INCLUDE_DIRS})
add_definitions(-DSENSORS_MOD_AVAILABLE)
add_definitions(-DVIDEO_MOD_AVAILABLE)
add_subdirectory(./zed-open-capture)
# add_definitions(-DSENSORS_MOD_AVAILABLE)
# add_definitions(-DVIDEO_MOD_AVAILABLE)
add_executable(${PROJECT_NAME} src/main.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
include_directories(${PROJECT_NAME} 
  ./ 
  ./zed-open-capture/include 
  ${LibUSB_INCLUDE_DIRS} 
  ${HIDAPI_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${OpenCV_LIBS}
  jsonl-recorder
  zed_open_capture
  # ./zed-open-capture/build/libzed_open_capture.so
)
