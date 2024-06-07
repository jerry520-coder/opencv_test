cmake_minimum_required(VERSION 3.10)
project(FindContours)


# 查找OpenCV库
find_package(OpenCV REQUIRED)
# 找到 gflags 库
find_package(gflags REQUIRED)
# 找到glog包
find_package (glog 0.7.0 REQUIRED)

# 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Debug") #需要这个，不然不会出现调试信息



# 设置源文件
set(SRC src/path.cpp)

# 指定生成目标
add_executable(path ${SRC})

# 链接OpenCV库
target_link_libraries(path ${OpenCV_LIBS})

# 链接 gflags 库
target_link_libraries(path gflags)
# 链接glog库
target_link_libraries(path glog::glog)

# 如果使用的是 GCC，可能需要添加以下链接选项
if (CMAKE_COMPILER_IS_GNUCXX)
    target_link_libraries(path stdc++fs)
endif()
