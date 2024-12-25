cmake_minimum_required(VERSION 3.10)
project(fkws)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 DART 库   /usr/local/lib/libdart-external-odelcpsolver.so.6.15.0   /usr/local/share/dart/cmake/dart_external-odelcpsolverTargets.cmake
find_package(DART 6.14 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)

# 查找 OctoMap 库 
find_package(octomap REQUIRED)

#find_package(octovis REQUIRED)

# 设置包含目录
include_directories(
    ${DART_INCLUDE_DIRS}       # DART 头文件路径
    /usr/include/eigen3        # Eigen 头文件路径
    ${OCTOMAP_INCLUDE_DIRS}    # OctoMap 头文件路径
    #${OCTOVIS_INCLUDE_DIRS}    # Octovis 头文件路径
)

# 设置库文件路径
link_directories(
    ${DART_LIBRARY_DIRS}       # DART 库文件路径
    /usr/lib/x86_64-linux-gnu  # OctoMap 库文件路径
    #/usr/local/lib    # Octovis 库文件路径
)

# 添加源文件
file(GLOB SOURCES "src/*.cpp")

# 创建可执行文件
add_executable(fkws ${SOURCES})

# 链接依赖库
target_link_libraries(fkws 
    ${DART_LIBRARIES}          # DART 库
    assimp                     # DART 的 Assimp 依赖库
    fcl                        # 碰撞检测库
    ccd                        # 碰撞检测库
    m                          # 数学库
    ${OCTOMAP_LIBRARIES}       # OctoMap 库
    #${OCTOVIS_LIBRARIES}       # Octovis 库
)
