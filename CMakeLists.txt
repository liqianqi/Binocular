# CMake 最低版本要求
cmake_minimum_required(VERSION 3.10) #old
#project(Energy)
project(Energy)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
# 库文件的放置目录
set(OpenCV_DIR /home/liqianqi/opencv-3.4.16/build)

# 开启 DeBug 调试
SET(CMAKE_BUILD_TYPE RELEASE)
#set(SDK_DIR /home/liqianqi/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2104.9022/Galaxy_camera/lib/x86_64/libgxiapi.so)

# C++11 标准、o3优化、多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")

find_package(OpenCV REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")#new
include_directories(${OpenCV_INCLUDE_DIRS} )

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
aux_source_directory(./src SRC_FILES)


## 指定生成目标
add_executable(${PROJECT_NAME} ${SRC_FILES})
add_definitions(-std=c++14 -O3)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})#new
#target_link_libraries(${PROJECT_NAME} ${SDK_DIR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ZED_LIBS})
