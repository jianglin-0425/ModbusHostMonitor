#pragma once
#include <QtGlobal>
#include <QByteArray>
#include <QString>

// Modbus 数据类型 (snapshotEntry::type 用同一套编码)
enum class RegType:quint8 { U16 = 0,I16 =1,U32 = 2,I32=3,F32=4};

// 寄存器区 (modbus 四个数据区)
enum class RegSpace : quint8
{
    Holding = 0 ,   // FC03 读 / FC06,16 写。  （保持寄存器：可读可写，通常是 16 位数据。对应功能码 03（读）和 06/16（写）
    Input   = 1,    // FC04 只读。             （输入寄存器）：只读。通常是传感器测量的实时值。对应功能码 04（读）。
    Coil    = 2,    // FC01 读 / FC05 写，位   （线圈）：位（1 Bit） 数据。可读可写。通常用来控制继电器开关（0关，1开）。对应功能码 01（读）和 05（写）
    Discrete= 3,    // FC02 只读， 位          （离散输入）：位（1 Bit） 数据。只读。通常用来读开关的物理状态。对应功能码 02（读）
};

/**
 * 判断一个给定的寄存器区域，是不是“位（Bit）”类型。
 *      1.Holding（保持寄存器） 和 Input（输入寄存器）：
 *              它们的数据单位是 16 位（2个字节）。你读回来的是一个整数，比如 0x0064（代表 100）。
 *      2. Coil（线圈） 和 Discrete（离散输入）：
 *              它们的数据单位是 1 位（1个 bit）。你读回来的要么是 0（False/断开），要么是 1（True/闭合）
 *
 * @param s
 * @return
 */
inline bool isBitSpace(RegSpace s){
    return s==RegSpace::Coil || s== RegSpace::Discrete;
}

inline RegSpace regSpaceFromString(const QString& s)
{
    if(s=="input")  return RegSpace::Input;
    if(s=="coil")   return RegSpace::Coil;
    if(s=="discrete")   return RegSpace::Discrete;
    return RegSpace::Holding;
}
inline QString regSpaceToString(RegSpace s){
    switch (s) {
        case RegSpace::Input:   return "input";
        case RegSpace::Coil:    return "coil";
        case RegSpace::Discrete:return "discrete";
        default:                return "holding";
    }
}

inline int regCount(RegType t)
{
    return (t==RegType::U16 || t== RegType::I16)?1:2;
}

inline RegType regTypeFromString(const QString& s)
{
    if(s == "i16")  return RegType::I16;
    if(s=="u32")    return RegType::U32;
    if(s=="i32")    return RegType::I32;
    if(s=="f32")    return RegType::F32;
    return RegType::U16;
}

inline QString regTypeToString(RegType t)
{
    switch (t) {
        case RegType::I16:  return "i16";
        case RegType::U32:  return "u32";
        case RegType::I32:  return "i32";
        case RegType::F32:  return "f32";
        default:            return "u16";
    }
}

// Modbus 功能码 / 异常码
namespace mb{

constexpr quint8 FC_READ_COILS      =   0x01;
constexpr quint8 FC_READ_DISCRETE   =   0x02;
constexpr quint8 FC_READ_HOLDING    =   0x03;
constexpr quint8 FC_READ_INPUT      =   0x04;
constexpr quint8 FC_WRITE_COIL      =   0x05;
constexpr quint8 FC_WRITE_SINGLE    =   0x06;
constexpr quint8 FC_WRITE_MULTIPLE  =   0x10;

inline quint8 readFuncFor(RegSpace s)
{
    switch (s) {
        case RegSpace::Input:   return FC_READ_INPUT;
        case RegSpace::Coil:    return FC_READ_COILS;
        case RegSpace::Discrete:return FC_READ_DISCRETE;
        default:                return FC_READ_HOLDING;
    }
}

constexpr int    EXC_ILLEGAL_FUNCTION = 1;
constexpr int    EXC_ILLEGAL_ADDRESS  = 2;
constexpr int    EXC_ILLEGAL_VALUE    = 3;
constexpr int    SLAVE_DEVICE_FAILURE    = 4;

inline QString excString(int code)
{
    switch (code) {
        case EXC_ILLEGAL_FUNCTION: return "illegal function";
        case EXC_ILLEGAL_ADDRESS:  return "illegal data address";
        case EXC_ILLEGAL_VALUE:    return "illegal data value";
        case SLAVE_DEVICE_FAILURE: return "slave device failure";
        default:                   return QString("exception %1").arg(code);
    }
}

// Modbus 的铁律：在总线上传输时，16 位寄存器永远是高字节在前（大端 / Big-Endian）。
// 大端 16bit 读写（寄存器一律大端）
/**
 * 把从网线（或串口）上收到的 2 个独立的字节（8位），按照 Modbus 的规则，拼装成一个完整的 16 位整数（寄存器数据）
 *
 * Example:
 * 假设传感器测到的温度是 0x1234（十六进制，十进制是 4660）。由于 Modbus 规定高位在前（大端序），网线上传过来的字节顺序是这样的：
 * 第 1 个字节 p[0] = 0x12
 * 第 2 个字节 p[1] = 0x34
 *      p[0] << 8：把第一个字节 0x12 向左移动 8 位，变成 0x1200（高 8 位就位）。
 *      | p[1]：把第二个字节 0x34 塞进低 8 位。0x1200 | 0x34 完美变成 0x1234。
 * @param p
 * @return
 */
inline quint16 beU16(const quint8* p)            { return quint16((p[0] << 8) | p[1]); }

inline void putBeU16(QByteArray& a,quint16 v){
    a.append(char(v >> 8));     // 0x12
    a.append(char(v & 0xFF));   // 0x34
}

// 32bit/float：两个寄存器，默认字序 ABCD（高字在前）
inline quint32 combineU32(quint16 hi,quint16 lo)
{
    return (quint32(hi)<<16)|lo;
}

inline double decodeValue(RegType t,quint16 r0,quint16 r1)
{
    switch (t) {
        case RegType::U16:  return double(r0);
        case RegType::I16:  return double(quint16(r0));
        case RegType::U32:  return double(combineU32(r0,r1));
        case RegType::I32:  return double(quint32(combineU32(r0,r1)));
        case RegType::F32:  {
            quint32 raw = combineU32(r0,r1);
            float f;
            static_assert(sizeof(f) == sizeof(raw), "float must be 32bit");
            memcpy(&f,&raw,sizeof(f));
            return double(f);
        }
    }
    return 0.0;
}
} // namespace mb