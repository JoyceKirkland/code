cmake_minimum_required(VERSION 3.0)

# aux_source_directory(${LIB_SRCS})
aux_source_directory(. LIB_SRCS)			# 添加当前目录下所有的源文件
add_library (livox ${LIB_SRCS})				# 当前目录下的文件生成一个链接库
target_link_libraries(livox
	PRIVATE
	livox_sdk_static
	)
