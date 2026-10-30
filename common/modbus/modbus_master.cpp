#include "modbus_master.h"
#include "crc16.h"

using namespace mb;

// ------------- 组帧
QByteArray ModbusMaster::wrap(quint8 slave,const QByteArray& pdu)
{
    QByteArray frame;
    if(m_mode == Mode::Rtu)
    {
        frame.append(char(slave));  // 从站
        frame.append(pdu);          // 协议数据单元
        quint16 crc = crc16(reinterpret_cast<const quint8*>(frame.constData()),
                            size_t(frame.size()));
        frame.append(char(crc & 0xFF)); // CRC 低字节在前
        frame.append(char(crc>>8));
    }
    else
    {
        m_lastTid = ++m_tid;
        putBeU16(frame,m_lastTid);      // 事务标识
        putBeU16(frame,0);              // 协议标识
        putBeU16(frame,quint16(1+pdu.size()));//长度 = unit + PDU
        frame.append(char(slave));      // 单元标识
        frame.append(pdu);
    }
    return frame;
}

QByteArray ModbusMaster::buildRead(RegSpace space, quint8 slave, quint16 start, quint16 count)
{
    QByteArray pdu;
    pdu.append(char(readFuncFor(space)));
    putBeU16(pdu,start);
    putBeU16(pdu,count);
    return wrap(slave,pdu);
}

QByteArray ModbusMaster::buildWriteSingle(quint8 slave, quint16 addr, quint16 value)
{
    QByteArray pdu;
    pdu.append(char(FC_WRITE_SINGLE));
    putBeU16(pdu,addr);
    putBeU16(pdu,value);
    return wrap(slave,pdu);
}

QByteArray ModbusMaster::buildWriteCoil(quint8 slave, quint16 addr, bool on)
{
    QByteArray pdu;
    pdu.append(char(FC_WRITE_COIL));
    putBeU16(pdu,addr);
    putBeU16(pdu,on ? 0xFF00:0x0000);           // 规范规定的两个魔数
    return wrap(slave,pdu);
}

QByteArray ModbusMaster::buildWriteMultiple(quint8 slave, quint16 start, const QVector<quint16> &vals)
{
    QByteArray pdu;
    pdu.append(char(FC_WRITE_MULTIPLE));
    putBeU16(pdu,start);
    putBeU16(pdu,quint16(vals.size()));
    pdu.append(char(vals.size() * 2));
    for (quint16 v:vals) {
        putBeU16(pdu,v);
    }
    return wrap(slave,pdu);
}

// ---------- 解析 ----------
/**
 * 负责剥去外壳（CRC 或 TCP 头
 * @param frame
 * @param slave
 * @param pdu
 * @param errCode
 * @return
 */
bool ModbusMaster::unwrap(const QByteArray &frame, quint8 slave, QByteArray &pdu, int &errCode) const
{
    errCode = 0;
    const auto* p = reinterpret_cast<const quint8*>(frame.constData());

    if(m_mode == Mode::Rtu){
        // 1.最短的报文是异常响应（设备地址 1B + 功能码 1B + 异常码 1B + CRC 2B = 5B）
        if(frame.size() < 5) return false;              // 最短：异常响应 5 字节

        // calc: 把你收到的这串数据，掐掉最后 2 个字节（frame.size()-2），扔进你之前写的 crc16 查表函数里，算出一个指纹。
        quint16 calc = crc16(p,size_t(frame.size()-2));
        // recv: 把最后 2 个字节提取出来。注意！低字节在前（p[size-2] | (p[size-1] << 8)），这是 Modbus RTU 的铁律。
        quint16 recv = quint16(p[frame.size() - 2] | (p[frame.size() - 1] << 8));
        if(calc != recv) return false; // 只要 calc != recv，说明数据在路上被干扰了，直接 return false。
        if(p[0] !=slave) return false; // 从站地址验证：p[0] 是设备地址。如果不是你要找的那个设备（slave），直接丢弃。

        // 提取 PDU：frame.mid(1, frame.size() - 3)。意思是从第 1 个字节开始（跳过设备地址），取 总长度 - 3 个字节（因为要去掉 1 字节地址和 2 字节 CRC），剩下的就是干净的功能码 + 数据（PDU
        pdu = frame.mid(1,frame.size() - 3);
    }else
    {
        // TCP 最少是 7 字节的 MBAP 头 + 1 字节功能码 = 8 字节。
        if(frame.size()<8) return false;    // 去掉 slave 和 crc

        // beU16(p) 取出前 2 个字节（事务标识符），必须等于你发请求时存下的 m_lastTid。这就防止了多个并发连接时“张冠李戴”。
        if(beU16(p) != m_lastTid) return false;

        // 协议号校验：beU16(p+2) 读出协议标识符，必须是 0。如果不是 0，说明这根本不是标准的 Modbus 报文
        if(beU16(p+2)!=0 ) return false;
        // 长度校验（核心防粘包）：len = beU16(p+4) 读出 MBAP 头里的长度字段。总长度必须是 6 + len（前 6 个字节 + 单元标识 + 后面的所有内容）。如果对不上，说明不是半包就是粘包，直接拒绝
        int len = beU16(p+4);

        // 提取 PDU：frame.mid(7)。因为 TCP 没有 CRC，只要跳过前 7 个字节的 MBAP 头，剩下的全是 PDU。
        if(frame.size() != 6+len)   return false;
        pdu = frame.mid(7);
    }
    // 最高位判断：pdu[0] & 0x80。Modbus 规定，如果设备报错，它会把功能码的最高位（第 7 位）置为 1。比如你发了 03（读保持寄存器），设备报错就会回 0x83。0x83 & 0x80 结果非 0，说明这是一包异常响应。
    if(pdu.isEmpty()) return false;
    // 此时，pdu[1] 就是异常码（比如 2 代表地址非法），把它赋给 errCode 带出去。
    if(quint8(pdu[0] & 0x80)){              // 异常响应
        if(pdu.size() < 2) return false;
        errCode = quint8(pdu[1]);
    }
    // 即使是异常响应，函数也返回 true。因为报文本身是合法的（CRC 或长度都对），只是业务上出错了。上层的业务代码会根据 errCode != 0 来弹出提示
    return true;
}
/**
 * parseReadResponse 负责校验业务逻辑、提取真正的 16 位数值
 * @param resp
 * @param slave
 * @param out
 * @param errCode
 * @return
 */
bool ModbusMaster::parseReadResponse(const QByteArray &resp, quint8 slave, QVector<quint16> &out, int &errCode) const
{
    out.clear();
    QByteArray pdu;
    if(!unwrap(resp,slave,pdu,errCode))
        return false;

    // 如果 errCode 不为 0，说明设备虽然收到了请求，但拒绝了执行（比如你读的地址不存在）
    if(errCode != 0)
        return true;

    // 03（读保持）或 04（读输入）
    const quint8 func = quint8(pdu[0]);
    if((func != FC_READ_HOLDING && func != FC_READ_INPUT) || pdu.size() < 2)
        return false;

    // pdu[0] 是功能码，pdu[1] 是后续数据的字节数。
    int byteCount = quint8(pdu[1]);
    if(pdu.size()!=2+byteCount || byteCount%2 !=0)
        return false;

    // 把指针 d 指向数据区的开头（跳过前 2 个字节的“功能码+字节数”）。
    //每次跳过 2 个字节（i+=2），调用你之前写的大端转换函数 beU16(d+i)，把两个 quint8 拼成一个 quint16
    const auto* d = reinterpret_cast<const quint8*>(pdu.constData())+2;
    for (int i = 0; i < byteCount; i+=2) {
        out.append(beU16(d+i));
    }
    /*
        . 为什么发送的时候也要转换？（putBeU16）
        当你的程序要下发一个指令，比如把寄存器改成 100（0x0064）。
        你的电脑内存里存的顺序是小端的 [0x64] [0x00]。
        如果你直接把这俩字节发出去，设备收到的是 0x6400（25600）。

        Modbus 协议规定：在电线/网线上跑的数据，必须是大端（高位在前）。
        你的电脑 CPU 习惯：在内存里存数据，通常是小端（低位在前）。
        大端转换的目的：就是为了填平这个鸿沟。把网线上收到的大端字节流，翻译成电脑能正确理解的数值；把电脑里的数值，翻译成大端字节流发给设备。
     */

    return true;
}

/**
 * 解析 FC 01（读线圈） 和 FC 02（读离散输入） 的响应报文。
 * @param resp      接收到的原始完整报文（Response）
 * @param slave     从站设备地址（Slave ID）。
 * @param count     请求读取的位的数量
 * @param out       最终解析结果的容器
 * @param errCode   异常码
 * @return
 */
bool ModbusMaster::parseReadBits(const QByteArray &resp, quint8 slave, int count,
                                        QVector<bool> &out,int &errCode) const
 {
    out.clear();
    QByteArray pdu;
    if(!unwrap(resp,slave,pdu,errCode))
        return false;
     if (errCode != 0)
         return true;
     const  quint8  func = quint8(pdu[0]);
     if((func != FC_READ_COILS && func != FC_READ_DISCRETE)||pdu.size() < 2)
         return false;
     const int byteCount = quint8(pdu[1]);
     // pdu.size() != 2+byteCount：防止报文被截断或粘包（物理长度不对）。
     // byteCount != (count+7)/8：防止设备返回的数据量不对（逻辑长度不对）。
     if(pdu.size() != 2+byteCount || byteCount !=(count+7)/8) // ”向上取整公式”：(A + B - 1) / B。在这里 B 是 8
         return false;

    /*
        这个循环完美地将连续的内存字节，解构成了一个个独立的 bool 值（0 或 1）。
        🎯 实战避坑提醒（关于 Modbus 的“位序”）
        这段代码的解析完全符合标准 Modbus 协议。但如果你对接的是一些非标设备，可能会遇到坑：
        高低位反转：有些奇葩设备（或者特定的网关），会把一个字节里的 8 个位倒过来发（比如把 0x80 当成第 0 位）。如果你发现读上来的状态全反了，就需要把 i % 8 改成 7 - (i % 8)。
        末尾补齐问题：如果你读了 10 个位，设备返回了 2 个字节（16 个位）。最后 6 个位是设备用 0 补齐的（填充位）。你的 for 循环条件是 i < count，完美避开了这些无效的填充位，只提取你需要的 10 个状态。这也是这段代码写得很严谨的地方。
        总结：这个函数的实现非常干净利落。用 (count+7)/8 巧妙校验长度，用 i/8 和 i%8 完成极其高效的位运算拆包。对于你后面写 UI 界面，直接把 QVector<bool> 里的数据绑定到 QCheckBox 或指示灯上即可。
    */
     const auto* d = reinterpret_cast<const quint8 *>(pdu.constData()) +2;
     out.reserve(count);
     for (int i = 0; i < count; ++i) {
         out.append(d[i/8] >> ( i % 8 ) & 1); // ?????
     }
     return true;
 }

 bool ModbusMaster::parseWriteResponse(const QByteArray &resp, quint8 slave, int &errCode) const
 {
    QByteArray pdu;
    if(!unwrap(resp,slave,pdu,errCode))
        return false;
    if(errCode != 0 )
        return true;
    quint8  func = quint8(pdu[0]);
    if(func != FC_WRITE_COIL && func != FC_WRITE_SINGLE && func !=FC_WRITE_MULTIPLE)
        return false;
     return pdu.size() == 5;
 }

