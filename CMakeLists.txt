cmake_minimum_required(VERSION 3.0.0)
project(code VERSION 0.1.0)

# include(CTest)
# enable_testing()

# add_library(code code.cpp)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

# Specialize CMake build flags for different build types
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-deprecated")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Use standard c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(OpenCV "/home/joyce/workplace/opencv4.5/opencv-4.5.0/build")
# set(Livox "/home/joyce/workplace/livox/livox_sdk/Livox-SDK")
# Find and add opencv as required
find_package(TensorRT REQUIRED)
find_package(OpenCV 4  REQUIRED)
# find_package(Livox REQUIRED)
find_package(Ceres 2 REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA  REQUIRED)
find_package(PythonLibs 3.8 REQUIRED)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TensorRT_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
# include_directories(${Livox_INCLUDE_DIRS})

# include_directories(${PYTHON_INCLUDE_DIRS})
# include_directories("./")

link_libraries(${OpenCV_LIBS})
link_libraries(${EIGEN3_LIBS})
link_libraries(${TensorRT_LIBS})
link_libraries(${CERES_LIBS})
link_libraries(${Livox_LIBS})

# 添加cuda头文件
# include_directories(/usr/local/cuda-11.4/lib)
include_directories(/usr/include/opencv4/opencv2)

#Uart
# add_library(serial KCF/devices/serial/uart_serial.cpp)
# target_link_libraries(code serial)

aux_source_directory(. ALL_SRCS)			# 添加当前目录下所有的源文件
# name of executable file and path of source file
add_executable(code ${ALL_SRCS})

set(serial_SRCS
	KCf/devices/new_serial/src/serial.cc
	KCf/devices/new_serial/src/impl/unix.cc
	KCf/devices/new_serial/src/impl/list_ports/list_ports_linux.cc
	KCf/devices/new_serial/serial/serial.h
	KCf/devices/new_serial/serial/v8stdint.h
)
include_directories(KCf/devices/new_serial)
add_library(new_serial ${serial_SRCS})
target_link_libraries(code new_serial)


# opencv libraries
target_link_libraries(code 
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${CERES_LIBS}
	${TensorRT}
	${CUDA_INCLUDE_DIRS}
	cuda
	fmt::fmt
	nvonnxparser
)

include_directories(basic_pnp)
add_subdirectory(KCf/angle_solve)
target_link_libraries(code basic_pnp)

include_directories(capture)
add_subdirectory(radar/camera)
target_link_libraries(code capture)

# include_directories(kcf)
# add_subdirectory(KCF/kcf)
# target_link_libraries(code kcf)

# include_directories(serial)
# add_subdirectory(KCf/devices/serial)
# target_link_libraries(code serial)