# add all cpp files in src to the shared library
# file (GLOB files ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp) # group all .cpp in files
# foreach(file ${files})
#     add_library(${PROJECT_NAME} SHARED ${file})
#     message ( \ \ \ \ [ \ Load \ All \ SRCS \ ]  \ ${file}\ will\ be\ added\ to\ the \ shared \ library.)
# endforeach(file ${files})

add_library(${PROJECT_NAME} SHARED
            imu.cpp
            gui.cpp
            vslamMgr.cpp
            videoCapture.cpp
            frontEnd.cpp
            frame.cpp
            map.cpp
            feature2d.cpp
            landMark.cpp
            backEnd.cpp
            orbMatcher.cpp)
target_link_libraries(mySLAM ${THIRD_PARTY_LIBS})