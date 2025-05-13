cmake_minimum_required(VERSION 3.10)
project(ceres_test)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# gprof性能分析的编译选项
set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS} -pg")

# 查找必要的包
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(nonlinear_optimization Nonlinear_Optimization.cpp)
add_executable(curve_fitting Nonlinear_Curve_Fitting.cpp)
add_executable(gprof_demo gprof_demo.cpp)
add_executable(valgrind_demo valgrind_demo.cpp)
add_executable(performance_demo performance_demo.cpp)

# 链接Ceres库到nonlinear_optimization
target_link_libraries(nonlinear_optimization ${CERES_LIBRARIES})

# 链接Ceres和OpenCV库到curve_fitting
target_link_libraries(curve_fitting 
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)

# 为gprof_demo添加性能分析编译选项
target_compile_options(gprof_demo PRIVATE -pg)
target_link_options(gprof_demo PRIVATE -pg)

# 包含头文件目录
include_directories(
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)