cmake_minimum_required(VERSION 3.16)
project(ModbusMaster VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Network Sql)
# 可选模块分明开找： 一个缺失不拖垮其他组件的探测
find_package(Qt6 QUIET COMPONENTS SerialPort)
find_package(Qt6 QUIET COMPONENTS Widgets)
find_package(Qt6 QUIET COMPONENTS tests)

add_library(common STATIC
        common/modbus/crc16.cpp
        common/modbus/crc16.h)
target_include_directories(common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/common)
target_link_libraries(common PUBLIC Qt6::Core Qt6::Network)
