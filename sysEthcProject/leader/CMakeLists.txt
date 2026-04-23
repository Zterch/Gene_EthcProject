cmake_minimum_required(VERSION 3.16)

# 设置PROJECT_NAME变量
set(PROJECT_NAME leader)

# 设置 EtherCAT 路径
set(ETHERCAT_PATH /usr/local/ethercat)

# 设置工程名
project (${PROJECT_NAME})

# 查找 SQLite3 包
find_package(SQLite3 REQUIRED)

# 查找 EtherCAT 库
find_library(ETHERCAT_LIB ethercat PATHS ${ETHERCAT_PATH}/lib REQUIRED)

# 添加包含目录
include_directories(include)
include_directories(../sys_share)
include_directories(${ETHERCAT_PATH}/include)
include_directories(${SQLite3_INCLUDE_DIRS})  # 确保包含 SQLite3 头文件路径

# 添加库目录
link_directories(${ETHERCAT_PATH}/lib)

# 查找当前目录下的所有源文件并存入DIR_SRCS变量
aux_source_directory(src DIR_SRCS)
aux_source_directory(../sys_share DIR_SRCS)
message(${DIR_SRCS})

# 添加一个可编译的目标到工程
add_executable(${PROJECT_NAME} ${DIR_SRCS})

# 链接库 - 确保使用 ${SQLite3_LIBRARIES} 而不是 sqlite3
target_link_libraries(${PROJECT_NAME} pthread m ${SQLite3_LIBRARIES} ${ETHERCAT_LIB})