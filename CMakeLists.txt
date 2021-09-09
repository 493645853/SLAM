#CmakeLists.txt
# CMAKE最低版本
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
# 项目名称
set(PROJECT_NAME slam_Project)
PROJECT(${PROJECT_NAME})

# 设定OpenCV编译好的OpenCVConfig.cmake地址, 只需要设置这里即可
SET("OpenCV_DIR" "D:/opencv/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#查找当前目录所有源文件
#并保存名称到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

#指定生成目标
add_executable(${PROJECT_NAME} ${DIR_SRCS})

#链接库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})