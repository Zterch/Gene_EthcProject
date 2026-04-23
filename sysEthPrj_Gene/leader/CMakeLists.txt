cmake_minimum_required(VERSION 3.16)

set(PROJECT_NAME leader)  
project(${PROJECT_NAME} C)

# 设置 C 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 设置 EtherCAT 路径
set(ETHERCAT_INCLUDE_DIR "/usr/local/ethercat/include")
set(ETHERCAT_LIBRARY_DIR "/usr/local/ethercat/lib")

# 检查路径是否存在
if(NOT EXISTS ${ETHERCAT_INCLUDE_DIR})
    message(WARNING "EtherCAT include directory not found: ${ETHERCAT_INCLUDE_DIR}")
else()
    message(STATUS "EtherCAT include directory: ${ETHERCAT_INCLUDE_DIR}")
endif()

# 查找库文件
find_library(ETHERCAT_LIB
    NAMES ethercat libethercat.so libethercat.a
    PATHS ${ETHERCAT_LIBRARY_DIR}
    NO_DEFAULT_PATH
)

if(NOT ETHERCAT_LIB)
    message(FATAL_ERROR "EtherCAT library not found in: ${ETHERCAT_LIBRARY_DIR}")
else()
    message(STATUS "Found EtherCAT library: ${ETHERCAT_LIB}")
endif()

# 设置包含目录
include_directories(
    include
    ../sys_share
    /opt/etherlab/include
    ${ETHERCAT_INCLUDE_DIR}
)

# 查找源文件
aux_source_directory(src DIR_SRCS)
aux_source_directory(../sys_share DIR_SRCS)

message(STATUS "Source files: ${DIR_SRCS}")

# 添加可执行文件
add_executable(${PROJECT_NAME} ${DIR_SRCS})

# 添加编译选项
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter  # 抑制未使用参数警告
    -D_GNU_SOURCE
    -D_REENTRANT
    -pedantic
)

# 链接库 - 注意顺序很重要
target_link_libraries(${PROJECT_NAME}
    #${ETHERCAT_LIB}  # 使用 find_library 找到的库
    -L/opt/etherlab/lib
    -lethercat
    -lpthread
    -lrt
    -lm
)


# 添加链接选项（如果需要）
set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_FLAGS "-Wl,--no-as-needed"  # 确保链接所有库
)