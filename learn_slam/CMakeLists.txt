#CmakeLists.txt
# CMAKE最低版本
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# 设定OpenCV编译好的OpenCVConfig.cmake地址, 只需要设置这里即可
SET("OpenCV_DIR" "D:/opencv/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#link_directories(${OpenCV_LIB_DIRS})
message(opencv \ includepath \ : \ ${OpenCV_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# sophus (只有头文件)
find_package(Sophus REQUIRED)
#指定头文件目录
set(Sophus_INCLUDE_DIRS "D:/SelfLearning_vscode/3rdProjects/Sophus")
include_directories(${Sophus_INCLUDE_DIRS})
message(sophus \ includepath \ : \ ${Sophus_INCLUDE_DIRS})

#分别接入不同的main 函数
# 遍历项目根目录下所有的 .cpp 文件
file (GLOB_RECURSE files *.cpp)
foreach (file ${files})
string(REGEX REPLACE ".+/(.+)\\..*" "\\1" exe ${file})
add_executable (${exe} ${file})
target_link_libraries(${exe} ${OpenCV_LIBS} ${PCL_LIBRARIES})
message ( \ \ \ \ [ \ Load \ All \ Mains \ ]  \ ${exe}.cpp\ will\ be\ compiled\ to\ ${exe})
endforeach ()