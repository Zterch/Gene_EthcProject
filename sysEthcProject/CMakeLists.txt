cmake_minimum_required(VERSION 3.16)
#add_definitions(-std=c++11)

project (sysEthcProject)

#守护进程
add_subdirectory(supr)
#添加模块
add_subdirectory(leader) 




