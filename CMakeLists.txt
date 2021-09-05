#CmakeLists.txt
# CMAKE最低版本
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
# 项目名称
PROJECT(MyProject)

#包含第三方库
include_directories("../3rdParty/opencv/include")
link_directories("../3rdParty/opencv/x64/lib")

#查找当前目录所有源文件
#并保存名称到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

#指定生成目标
add_executable(MyProject ${DIR_SRCS})

