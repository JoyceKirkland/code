find_package(OpenCV 4 REQUIRED)
find_package(fmt REQUIRED)
aux_source_directory(. LIB_SRCS)			# 添加当前目录下所有的源文件
add_library (capture ${LIB_SRCS})				# 当前目录下的文件生成一个链接库
target_link_libraries(capture 
	${OpenCV_LIBS}
	/home/joyce/downloads/linuxSDK_V2.1.0.27/lib/x64/libMVSDK.so
	fmt::fmt
)