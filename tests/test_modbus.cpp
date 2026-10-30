#include "modbus/crc16.h"
#include "modbus/modbus_master.h"

#include <QtTest/QtTest>

class TestModbus:public QObject
{
    Q_OBJECT
private slots:
    // ---- CRC16 ----
    void crc16_knownVector()
    {
        // 读保持：01 03 00 00 00 0A 的 CRC = 0xCDC5（线上低字节在前：C5 CD）
        const quint8 frame[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0A };
        QCOMPARE(crc16(frame, sizeof(frame)), quint16(0xCDC5));
    }

    // ---- RTU 组帧 ----
    // CRC 已由 crc16_knownVector 单独验证，组帧测试断言帧结构 + CRC 自洽
    static bool crcOK(const QByteArray& f)
    {
        const auto* p =reinterpret_cast<const quint8*>(f.constData());
        const quint16 calc = crc16(p,size_t(f.size()-2));
        return quint8(f[f.size() - 2]) == (calc & 0xFF) // 低字节在前
            && quint8(f[f.size()-1])== (calc >> 8);
    }

    void rtu_buildReadHolding()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        const QByteArray f = m.buildReadHolding(1,0,10);
        QCOMPARE(f.toHex(),QByteArray("01030000000ac5cd"));     // 规范已知向量
    }

    void rtu_buildWriteSingle()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        const QByteArray f = m.buildWriteSingle(1,0x0001,0x0003);
        QCOMPARE(f.left(6).toHex(),QByteArray("010600010003"));
        QCOMPARE(f.size(),8);
        QVERIFY(crcOK(f));
    }
    void rtu_buildWriteMultiple()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        const QByteArray f = m.buildWriteMultiple(1,0,{0x000A,0x0102});
        QCOMPARE(f.left(11).toHex(),QByteArray("01100000000204000a0102"));
        QCOMPARE(f.size(),13);
        QVERIFY(crcOK(f));
    }
    // ---- RTU 解析 ----
    void rtu_parseReadResponse()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        // 01 03 04 00 6B 00 05 + 正确 CRC：手工算
        QByteArray resp = QByteArray::fromHex("010304006b0005");
        const quint16  crc = crc16(reinterpret_cast<const quint8*>
                                 (resp.constData()),size_t(resp.size()));

        resp.append(char(crc & 0xFF)).append(char(crc>>8));
        QVector<quint16> out;
        int exc = -1;
        QVERIFY(m.parseReadResponse(resp,1,out,exc));
        QCOMPARE(exc,0);
        QCOMPARE(out,(QVector<quint16>{0x006B,0x0005}));
    }
    void rtu_parseException()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        QByteArray resp = QByteArray::fromHex("018302");        // 0x|0x80 + 异常码 02
        const quint16 crc = crc16(reinterpret_cast<const quint8*> (resp.constData()),size_t(resp.size()));
        resp.append(char(crc & 0xFF)).append(char(crc >> 8));
        QVector<quint16 > out;
        int exc = 0;
        QVERIFY(m.parseReadResponse(resp,1,out,exc));
        QCOMPARE(exc,2);                // 不合法的数据地址
    }
    void rtu_rejectBadCrc()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        const QByteArray resp = QByteArray::fromHex("010302006bffff");      // CRC 错误
        QVector<quint16> out;
        int exc = 0;
        QVERIFY(!m.parseReadResponse(resp,1,out,exc));
    }

    // ---- TCP（MBAP）----
    void tcp_buildAndParse()
    {
        ModbusMaster m(ModbusMaster::Mode::Tcp);
        const QByteArray req = m.buildReadHolding(1,0,2);
        QCOMPARE(req.size(),12);        // MBAP 7 + PDU 5
        QCOMPARE(req.mid(2,2),QByteArray::fromHex("0000"));  // 长度 = unit + PDU
        QCOMPARE(req.mid(4,2),QByteArray::fromHex("0006"));     // 成都 = unit + PDU

        // 构造回带同义事务号的响应
        QByteArray resp = req.left(2);
        resp += QByteArray::fromHex("0000 0007 01 03 04 006b 0005");
        QVector<quint16> out;
        int exc = -1;
        QVERIFY(m.parseReadResponse(resp,1,out,exc));
        QCOMPARE(exc,0);
        QCOMPARE(out,(QVector<quint16 >{0x006B,0x0005}));
    }

    void tcp_rejectWrongTid()
    {
        ModbusMaster m(ModbusMaster::Mode::Tcp);
        m.buildReadHolding(1,0,2);
        const QByteArray resp = QByteArray::fromHex("ffff 0000 0007 01 03 04 006b  0005");
        QVector<quint16> out;
        int exc = 0;
        QVERIFY(!m.parseReadResponse(resp,1,out,exc));  // 事务号 不符必须拒收
    }
    // ---- FC 01/02/04/05 (P0 工程化扩展) ------------
    void rtu_buildReadCoils()
    {
        ModbusMaster m (ModbusMaster::Mode::Rtu);
        // 规范示例: 读从站 1 线圈 19 起 19搁 ——> 01 01 0013 0013
        const QByteArray f = m.buildRead(RegSpace::Coil,1,19,19);
        QCOMPARE(f.left(6).toHex(),QByteArray("010100130013"));
        QVERIFY(crcOK(f));
    }
    void rtu_parseReadBits()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        m.buildRead(RegSpace::Coil,1,0,10);
        // 响应: 01 01 02 b5 02 -- 10 位 = CD 风格位序 LSB 在前
        QByteArray resp = QByteArray::fromHex("010102b502");
        const quint16  crc = crc16(reinterpret_cast<const quint8 *>(resp.constData()),
                                   size_t(resp.size()));

        resp.append(char(crc &0xFF)).append(char(crc >> 8));

        QVector<bool> bits;
        int exc = -1;
        QVERIFY(m.parseReadBits(resp,1,10,bits,exc));
        QCOMPARE(exc ,0);

        // 0xb5 = 1011 0101b，LSB 在前 → 位 0..7 = 1,0,1,0,1,1,0,1；0x02 → 位 8,9 = 0,1
        QCOMPARE(bits, (QVector<bool>{ 1, 0, 1, 0, 1, 1, 0, 1, 0, 1 }));
    }
    void rtu_buildReadInput()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        const QByteArray f = m.buildRead(RegSpace::Input,1,0,2);
        QCOMPARE(f.left(6).toHex(),QByteArray("010400000002"));

        // FC04 响应与 FC03 同构，parseReadResponse 应能解析
        QByteArray resp = QByteArray::fromHex("010404006b0005");
        const quint16 crc = crc16(reinterpret_cast<const quint8*>(resp.constData()),
                                  size_t(resp.size()));
        resp.append(char(crc & 0xFF)).append(char(crc >> 8));
        QVector<quint16> out;
        int exc = -1;
        QVERIFY(m.parseReadResponse(resp, 1, out, exc));
        QCOMPARE(out, (QVector<quint16>{ 0x006B, 0x0005 }));
    }

    void rtu_buildWriteCoil()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        const QByteArray on = m.buildWriteCoil(1, 3, true);
        QCOMPARE(on.left(6).toHex(), QByteArray("01050003ff00"));
        QVERIFY(crcOK(on));
        const QByteArray off = m.buildWriteCoil(1, 3, false);
        QCOMPARE(off.left(6).toHex(), QByteArray("010500030000"));
        QVERIFY(crcOK(off));
        // FC05 响应是请求回显，parseWriteResponse 应接受
        int exc = -1;
        QVERIFY(m.parseWriteResponse(on, 1, exc));
        QCOMPARE(exc, 0);
    }

    void expectedLen_newFuncs()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        QCOMPARE(m.expectedLen(QByteArray::fromHex("010102")), 7);    // FC01: 3+2+2
        QCOMPARE(m.expectedLen(QByteArray::fromHex("010404")), 9);    // FC04: 3+4+2
        QCOMPARE(m.expectedLen(QByteArray::fromHex("0105")), 8);      // FC05 定长
    }

    // ---- 预期长度收帧 ----
    void expectedLen_rtu()
    {
        ModbusMaster m(ModbusMaster::Mode::Rtu);
        QCOMPARE(m.expectedLen(QByteArray::fromHex("01")), -1);          // 不够判断
        QCOMPARE(m.expectedLen(QByteArray::fromHex("0103")), -1);        // 03 还缺 byteCount
        QCOMPARE(m.expectedLen(QByteArray::fromHex("010304")), 9);       // 3+4+2
        QCOMPARE(m.expectedLen(QByteArray::fromHex("0106")), 8);         // 写响应定长
        QCOMPARE(m.expectedLen(QByteArray::fromHex("0183")), 5);         // 异常定长
    }

    void expectedLen_tcp()
    {
        ModbusMaster m(ModbusMaster::Mode::Tcp);
        QCOMPARE(m.expectedLen(QByteArray::fromHex("0001000000")), -1);  // MBAP 没收齐
        QCOMPARE(m.expectedLen(QByteArray::fromHex("000100000006")), 12);
    }
};

QTEST_GUILESS_MAIN(TestModbus)
#include "test_modbus.moc"