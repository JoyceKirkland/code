# requirement of cmake version
cmake_minimum_required(VERSION 3.15)

# project name
PROJECT(opencv_test)

# Specialize CMake build flags for different build types
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-deprecated")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Use standard c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(OpenCV "/home/joyce/workplace/opencv4.5/opencv-4.5.0/build")
# Find and add opencv as required
# find_package(TensorRT REQUIRED)
find_package(OpenCV 4  REQUIRED)
# find_package(Ceres 2 REQUIRED)
find_package(fmt REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(CUDA  REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${TensorRT_INCLUDE_DIRS})
# include_directories(${CERES_INCLUDE_DIRS})
# include_directories(${CUDA_INCLUDE_DIRS})

link_libraries(${OpenCV_LIBS})
# link_libraries(${EIGEN3_LIBS})
# link_libraries(${TensorRT_LIBS})
# link_libraries(${CERES_LIBS})

# 添加cuda头文件
# include_directories(/usr/local/cuda-11.4/lib)
# include_directories(/usr/include/opencv4/opencv2)


aux_source_directory(. ALL_SRCS)			# 添加当前目录下所有的源文件
# name of executable file and path of source file
add_executable(opencv_test ${ALL_SRCS})

# opencv libraries
target_link_libraries(opencv_test 
	${OpenCV_LIBS}
	# ${EIGEN3_LIBS}
	# ${CERES_LIBS}
	# ${TensorRT}
	# ${CUDA_INCLUDE_DIRS}
	# cuda
	fmt::fmt
	# nvonnxparser
)

# include_directories(basic_pnp)
# add_subdirectory(angle_solve)
# target_link_libraries(opencv_test basic_pnp)

include_directories(capture)
add_subdirectory(camera)
target_link_libraries(opencv_test capture)

# include_directories(serial)
# add_subdirectory(serial)
# target_link_libraries(opencv_test serial)