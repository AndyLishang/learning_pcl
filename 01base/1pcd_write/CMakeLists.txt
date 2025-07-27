cmake_minimum_required(VERSION 2.6 FATAL_ERROR) # 对于cmake版本的最低版本的要求

project(pcd_write) #  建立的工程名，例如源代码目录路径的变量名为CH_DIR

find_package(PCL REQUIRED)  # 自动查找系统中安装的库及其依赖项
# 因为PCL是模块化的，也可以如下操作：
#   一个组件  find_package(PCL REQUIRED COMPONENTS  io)
#   多个组件  find_package(PCL REQUIRED COMPONENTS commom io)
#   所有组件  find_package(PCL REQUIRED )

message(STATUS "PCL Version: ${PCL_VERSION}")   # 打印查找到的PCL版本

add_executable(pcd_write pcd_write.cpp)     # 告诉CMake从单个源文件write_pcd建立一个可执行文件
#虽然包含了PCL的头文件，因此编译器知道我们现在访问所用的方法，我们也需要让链接器知道所链接的库，PCL找到库文件由
#PCL_COMMON_LIBRARIES变量指示，通过target_link_libraries这个宏来出发链接操作
target_link_libraries (pcd_write ${PCL_LIBRARIES}) #