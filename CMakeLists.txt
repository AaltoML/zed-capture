cmake_minimum_required(VERSION 3.0)
project(zed_capture)

option (USE_ZED_SDK "Additionally builds capture tool using ZED SDK to enable capturing pose" ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -O2 -lstdc++")
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

option (JSON_BuildTests "Build nlohmann_json unit tests" ON)
set(JSON_BuildTests OFF)
set(USE_OPENCV_VIDEO_RECORDING ON CACHE INTERNAL "")
add_definitions(-DUSE_OPENCV_VIDEO_RECORDING)
add_subdirectory("jsonl-recorder")

find_package(OpenCV 4 REQUIRED)
find_package(LIBUSB 1 REQUIRED)
find_package(HIDAPI REQUIRED)
add_definitions(-DSENSORS_MOD_AVAILABLE)
add_definitions(-DVIDEO_MOD_AVAILABLE)
add_subdirectory(./zed-open-capture)
add_executable(${PROJECT_NAME} src/zed_capture.cpp)
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
)

if (USE_ZED_SDK)

  option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

  if (NOT LINK_SHARED_ZED AND MSVC)
      message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
  endif()

  if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
  endif(COMMAND cmake_policy)

  find_package(ZED 3 REQUIRED)
  find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

  IF(NOT WIN32)
      SET(SPECIAL_OS_LIBS "pthread" "X11")
      add_definitions(-Wno-write-strings)
  ENDIF()
  
  include_directories(${ZED_INCLUDE_DIRS})
  include_directories(${CUDA_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  link_directories(${ZED_LIBRARY_DIR})
  link_directories(${CUDA_LIBRARY_DIRS})

  ADD_EXECUTABLE(zed_pose_capture src/zed_pose_capture.cpp)
  add_definitions(-std=c++14 -O3)

  if (LINK_SHARED_ZED)
      SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
  else()
      SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
  endif()

  set_target_properties(zed_pose_capture PROPERTIES PREFIX "")

  TARGET_LINK_LIBRARIES(zed_pose_capture 
    ${OpenCV_LIBS}
    jsonl-recorder
    ${SPECIAL_OS_LIBS} 
    ${ZED_LIBS} 
  )

endif()