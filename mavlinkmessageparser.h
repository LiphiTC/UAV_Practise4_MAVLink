#pragma once

#include <QByteArray>
#include <QString>
#include <array>

// Идентификаторы и значения из MAVLink common.xml.
namespace mavlink {
constexpr quint8 MAGIC_V1 = 0xFE;
constexpr quint8 MAGIC_V2 = 0xFD;
constexpr int MIN_PACKET_LEN = 12;
enum MessageId : quint32 {
    HEARTBEAT = 0,
    GLOBAL_POSITION_INT = 33,
    COMMAND_LONG = 76,
    COMMAND_ACK = 77
};
enum Command : quint16 {
    MAV_CMD_NAV_LOITER_UNLIM = 17,
    MAV_CMD_NAV_LAND = 21,
    MAV_CMD_NAV_TAKEOFF = 22
};
enum Result : quint8 {
    MAV_RESULT_ACCEPTED = 0,
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    MAV_RESULT_DENIED = 2,
    MAV_RESULT_UNSUPPORTED = 3,
    MAV_RESULT_FAILED = 4,
    MAV_RESULT_IN_PROGRESS = 5
};
enum State : quint8 {
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_CRITICAL = 5,
    MAV_STATE_EMERGENCY = 6
};
}

struct MavlinkPacket {
    quint8 sequence = 0;
    quint8 systemId = 0;
    quint8 componentId = 0;
    quint32 messageId = 0;
    QByteArray payload;
};
struct Heartbeat {
    quint32 customMode = 0;
    quint8 type = 2;
    quint8 autopilot = 0;
    quint8 baseMode = 0;
    quint8 systemStatus = mavlink::MAV_STATE_STANDBY;
    quint8 mavlinkVersion = 3;
};
// Координаты — 1e-7 градуса, высоты — мм, скорости — см/с (NED), курс — сотые градуса.
struct GlobalPositionInt {
    quint32 timeBootMs = 0;
    qint32 latitudeE7 = 0;
    qint32 longitudeE7 = 0;
    qint32 altitudeMm = 0;
    qint32 relativeAltitudeMm = 0;
    qint16 vx = 0;
    qint16 vy = 0;
    qint16 vz = 0;
    quint16 heading = 0;
};
struct CommandLong {
    std::array<float, 7> params{};
    quint16 command = 0;
    quint8 targetSystem = 0;
    quint8 targetComponent = 0;
    quint8 confirmation = 0;
};
struct CommandAck {
    quint16 command = 0;
    quint8 result = 0;
};

// Учебная реализация без сгенерированной библиотеки: формат кадра MAVLink 2,
// CRC-16/MCRF4XX и CRC_EXTRA совпадают с common.xml. Класс не знает про сокеты.
class MavlinkMessageParser {
public:
    static bool isValid(const QByteArray& data);
    // Пустая строка — пакет корректен, иначе причина отказа.
    static QString validationError(const QByteArray& data);
    static bool parse(const QByteArray& data, MavlinkPacket& packet, QString* error = nullptr);
    static QByteArray pack(const MavlinkPacket& packet);
    static quint16 crc16(const QByteArray& data, quint16 crc = 0xFFFF);
    static int crcExtra(quint32 messageId);

    static QByteArray encode(const Heartbeat& message);
    static QByteArray encode(const GlobalPositionInt& message);
    static QByteArray encode(const CommandLong& message);
    static QByteArray encode(const CommandAck& message);
    static Heartbeat decodeHeartbeat(const QByteArray& payload);
    static GlobalPositionInt decodeGlobalPosition(const QByteArray& payload);
    static CommandLong decodeCommandLong(const QByteArray& payload);
    static CommandAck decodeCommandAck(const QByteArray& payload);

    static QString flightModeName(quint32 customMode);
    static QString resultText(quint8 result);
    static QString commandName(int command);
    static QString stateName(quint8 state);
};
