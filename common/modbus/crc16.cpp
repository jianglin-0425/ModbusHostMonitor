#include "crc16.h"

namespace {
// 启动时一次性生成 0xA001 查找表，与打死在代码里的表等价，但是不会抄错
struct Crc16Table{
    quint16  t[256];
    Crc16Table(){
        for (int i = 0; i < 256; ++i) {
            quint16 crc = quint16(i);
            for (int b = 0; b < 8; ++b) {
                crc =(crc & 1)? quint16((crc >> 1)^ 0xA001 ):quint16(crc >> 1);
            }
            t[i] =crc;
        }
    }
};
const Crc16Table g_table;
}   // namespace

quint16 crc16(const quint8* data,size_t len)
{
    quint16 crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = quint16((crc >> 8) ^ g_table.t[(crc ^ data[i]) & 0xFF]);
    }
    return crc;
}

quint16 crc16_bitwise(const quint8* data, size_t len)
{
    quint16 crc = 0xFFFF;       // 1.初始值全为 1;
    for (size_t i = 0; i < len; ++i) {
        crc ^ data[i];      //  2. 把当前字节 xor 进寄存器低 8 位
        for (int j = 0; j < 8; ++j) {   // 3.处理八位
            if(crc & 0x0001)        // 4.如果最低位是 1
                crc = (crc >> 1) ^ 0xA001;  // 右移后 xor 多项式
            else
                crc >>= 1 ;      // 否则直接右移
        }
    }
    return crc;         // 最后的 crc 就是结果
}
