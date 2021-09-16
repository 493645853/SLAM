#CmakeLists.txt
# CMAKE最低版本
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
if(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# include directory
include_directories(
        ${PROJECT_SOURCE_DIR}/Include/
)

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

# ceres
find_package(Ceres CONFIG REQUIRED)

# g2o
find_package(g2o CONFIG REQUIRED)

# pangolin
find_package(Pangolin CONFIG REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#分别接入不同的main 函数
# 遍历项目根目录下所有的 .cpp 文件 (GLOB_RECURSE)
#file (GLOB_RECURSE files *.cpp)
# 遍历指定目录的.cpp文件 (GLOB)
file (GLOB files ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach (file ${files})
string(REGEX REPLACE ".+/(.+)\\..*" "\\1" exe ${file})
add_executable (${exe} ${file})
target_link_libraries(${exe} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${g2o_LIBS})
target_link_libraries(${exe} g2o::core g2o::stuff g2o::types_icp g2o::types_sba)
target_link_libraries(${exe} ceres Ceres::ceres)
target_link_libraries(${exe} pangolin)
message ( \ \ \ \ [ \ Load \ All \ Mains \ ]  \ ${exe}.cpp\ will\ be\ compiled\ to\ ${exe})
endforeach ()