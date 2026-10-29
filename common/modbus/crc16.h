#pragma once
#include <QtGlobal>
#include <cstddef>

/**
2. CRC16(crc16.h/.cpp)
RTU 帧尾的 CRC16-MODBUS(多项式 0xA001,初值 0xFFFF)。两种写法:
- 位循环版:8 行,慢(每字节 8 次移位);
- **查表版**:预生成 256 项表,每字节一次查表。采集是高频路径,直接写查表版。
表可以程序启动时生成(静态局部变量 + lambda 初始化),不用硬编码 256 个常量。
**单测先行**:找两组已知向量断言,比如空数据、标准帧 `01 03 00 00 00 02` 的 CRC 是 `C4 0B`
(注意:**CRC 低字节在前**拼到帧尾)。写错多项式或字节序,这两个用例立刻报。
*/

// Modbus RTU CRC16: 多项式 0xA001 (反射)，查表法
quint16 crc16(const quint8* data,size_t len);

quint16 crc16_bitwise(const quint8* data, size_t len);