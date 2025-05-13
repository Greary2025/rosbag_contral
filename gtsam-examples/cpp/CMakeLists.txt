# 安装头文件到系统include目录
# 使用GLOB收集所有.h文件到ex_headers变量
file(GLOB ex_headers "*.h")
install(FILES ${ex_headers} DESTINATION include/gtsamexamples)

# 单元测试部分(当前被注释掉)
#gtsamAddTestsGlob(tests "tests/*.cpp" "" ${PROJECT_NAME})

# 添加可执行示例程序
# 使用gtsam提供的宏添加examples目录下的所有cpp文件作为示例
gtsamAddExamplesGlob("examples/*.cpp" "" "${PROJECT_NAME}")


### 构建共享库 ###
# 只有当存在.cpp或.c文件时才构建库
# 使用*.cpp模式收集所有源文件
file(GLOB ex_srcs "*.cpp")

# 创建共享库目标
# 包含所有源文件和头文件
add_library(${PROJECT_NAME} SHARED ${ex_srcs} ${ex_headers})

# 链接依赖库
# 链接Boost库和gtsam库
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} gtsam)


