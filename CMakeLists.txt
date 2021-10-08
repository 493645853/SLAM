#CmakeLists.txt

#### CMAKE最低版本 ###
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
project(mySLAM)

### complier optimization ###
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++14 -O3 -fopenmp -pthread")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) # Executable bin opt
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) # output lib

### Libs ###
# Eigen
include_directories("/usr/include/eigen3")

# opencv 
find_package(OpenCV CONFIG REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# sophus
find_package(Sophus CONFIG REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# g2o
find_package(g2o CONFIG REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# pangolin
find_package(Pangolin CONFIG REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# glog
find_package(Glog CONFIG REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gflags
find_package(GFlags CONFIG REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# define the path name "THIRD_PARTY_LIBS" for the project
set(THIRD_PARTY_LIBS 
    ${Sophus_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${Pangolin_LIBRARIES} 
    ${g2o_LIBS} g2o::core g2o::stuff g2o::types_icp g2o::types_sba 
    ${pangolin} 
    ${GLOG_LIBRARIES} glog::glog 
    ${GFLAGS_LIBRARIES} 
    pthread
    )

    
### path configuration ###
include_directories(${PROJECT_SOURCE_DIR}/include) # include directory
add_subdirectory(app)
add_subdirectory(src)

