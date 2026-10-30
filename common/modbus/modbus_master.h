#pragma once
#include "modbus_types.h"
#include "QVector"

// 组帧/解析，与通道（串口/TCP）完全解耦。
// RTU：  [slave][func][data...][CRC16 低字节在前]
// TCP：  [事务 2B][协议 2B=0][长度 2B][unit][func][data...]，无 CRC

class ModbusMaster {
public:
    enum class Mode {Rtu,Tcp};

    explicit ModbusMaster(Mode mode): m_mode(mode) {}
    Mode mode() const{ return m_mode; }

    // ---------- 组帧 ----------
    // 按数据区读: holding -> 03 / input -> 04 / coil -> 01 / discrete -> 02
    QByteArray buildRead(RegSpace space,quint8 slave,quint16 start,quint16 count);
    QByteArray buildReadHolding(quint8 slave,quint16 start,quint16 count)
    {
        return buildRead(RegSpace::Holding,slave,start,count);
    }
    QByteArray buildWriteSingle(quint8 slave,quint16 addr,quint16 value);
    QByteArray buildWriteCoil(quint8 slave,quint16 addr,bool on);   //FC 05
    QByteArray buildWriteMultiple(quint8 slave,quint16 start,const QVector<quint16>& vals);

    // ------------ 解析 ----------------
    // 返回 true 表示解析出合法响应; errCode = 0 正常，非 0 为 Modbus 异常码。
    // 返回 false 表示帧不合法 (CRC 错 / 长度错 / 从站不符)
    // FC 03 /04 (16bit 寄存器)
    bool parseReadResponse(const QByteArray& resp,quint8 slave,
                           QVector<quint16>& out,int& errCode) const;
    // FC 01/02（位，LSB 在前按字节打包；count = 请求的位数）
    bool parseReadBits(const QByteArray& resp,quint8 slave,int count,
                       QVector<bool>& out,int & errCode) const;

    // FC 05/06/16
    bool parseWriteResponse(const QByteArray& resp,quint8 slave,int& errCode) const;

    // ---- 收帧：按「预期长度」判断帧是否收齐 ----
    // 给 IChannel::transaction 当回调： 传入已收到的字节
    // 返回期望总长： 还判断不出来返回 -1(继续收，靠超时兜底)
    int expectedLen(const QByteArray& soFar) const;

private:
    QByteArray wrap(quint8 slave,const QByteArray& pdu);        // 加 RTU CRC 或 TCP MBAP
    bool unwrap(const QByteArray& frame,quint8 slave,
                QByteArray& pdu,int& errCode) const;            // 检验并列出有 PDU

    Mode m_mode;
    quint16  m_tid = 0;     // TCP 事务号 (自增)
    quint16 m_lastTid = 0;  // 最近一次请求的事务号，响应校验用
};
