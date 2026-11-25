#include "mavlinkmessageparser.h"
#include <QDataStream>
#include <QIODevice>

namespace {
constexpr int HEADER_V1 = 6;
constexpr int HEADER_V2 = 10;
constexpr int SIGNATURE_LEN = 13;
constexpr quint8 INCOMPAT_SIGNED = 0x01;

quint8 byteAt(const QByteArray& data, int index) { return static_cast<quint8>(data.at(index)); }
// Полная длина полезной нагрузки вместе с расширениями MAVLink 2.
int payloadLength(quint32 messageId)
{
    switch (messageId) {
    case mavlink::HEARTBEAT: return 9;
    case mavlink::GLOBAL_POSITION_INT: return 28;
    case mavlink::COMMAND_LONG: return 33;
    case mavlink::COMMAND_ACK: return 10;
    }
    return -1;
}
template<typename Fill> QByteArray write(Fill fill)
{
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    fill(stream);
    return bytes;
}
// Усечённые нули в конце полезной нагрузки восстанавливаются перед чтением.
template<typename Fill> void read(QByteArray payload, quint32 messageId, Fill fill)
{
    const int length = payloadLength(messageId);
    if (payload.size() < length) payload.append(QByteArray(length - payload.size(), '\0'));
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    fill(stream);
}
}

quint16 MavlinkMessageParser::crc16(const QByteArray& data, quint16 crc)
{
    // CRC-16/MCRF4XX (X.25), как crc_accumulate() в mavlink/checksum.h.
    for (const char value : data) {
        quint8 tmp = static_cast<quint8>(value) ^ static_cast<quint8>(crc & 0xFF);
        tmp ^= static_cast<quint8>(tmp << 4);
        crc = static_cast<quint16>((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
    }
    return crc;
}
int MavlinkMessageParser::crcExtra(quint32 messageId)
{
    switch (messageId) {
    case mavlink::HEARTBEAT: return 50;
    case mavlink::GLOBAL_POSITION_INT: return 104;
    case mavlink::COMMAND_LONG: return 152;
    case mavlink::COMMAND_ACK: return 143;
    }
    return -1;
}
bool MavlinkMessageParser::isValid(const QByteArray& data) { return validationError(data).isEmpty(); }
QString MavlinkMessageParser::validationError(const QByteArray& data)
{
    if (data.size() < 8) return QString("Слишком короткий MAVLink-пакет: %1 байт").arg(data.size());
    const quint8 magic = byteAt(data, 0);
    if (magic != mavlink::MAGIC_V2 && magic != mavlink::MAGIC_V1)
        return QString("Неверный стартовый байт 0x%1").arg(magic, 2, 16, QChar('0'));
    const bool v2 = magic == mavlink::MAGIC_V2;
    if (v2 && data.size() < mavlink::MIN_PACKET_LEN)
        return QString("Слишком короткий MAVLink-пакет: %1 байт").arg(data.size());
    const int length = byteAt(data, 1);
    const int header = v2 ? HEADER_V2 : HEADER_V1;
    const quint8 incompat = v2 ? byteAt(data, 2) : 0;
    if (incompat & ~INCOMPAT_SIGNED)
        return QString("Неподдерживаемые флаги несовместимости 0x%1").arg(incompat, 2, 16, QChar('0'));
    const int expected = header + length + 2 + ((incompat & INCOMPAT_SIGNED) ? SIGNATURE_LEN : 0);
    if (data.size() != expected)
        return QString("Длина датаграммы %1 байт не совпадает с полем len (ожидалось %2)").arg(data.size()).arg(expected);
    const quint32 messageId = v2 ? byteAt(data, 7) | byteAt(data, 8) << 8 | byteAt(data, 9) << 16 : byteAt(data, 5);
    const int extra = crcExtra(messageId);
    if (extra < 0) return QString("Неподдерживаемое сообщение msgid=%1").arg(messageId);
    if (length > payloadLength(messageId))
        return QString("Неверная длина полезной нагрузки msgid=%1: %2 байт").arg(messageId).arg(length);
    // CRC считается без стартового байта и подписи, в конце добавляется CRC_EXTRA.
    quint16 crc = crc16(data.mid(1, header - 1 + length));
    crc = crc16(QByteArray(1, static_cast<char>(extra)), crc);
    const quint16 received = static_cast<quint16>(byteAt(data, header + length) | byteAt(data, header + length + 1) << 8);
    if (crc != received)
        return QString("Неверная контрольная сумма: 0x%1 вместо 0x%2")
            .arg(received, 4, 16, QChar('0')).arg(crc, 4, 16, QChar('0'));
    return {};
}
bool MavlinkMessageParser::parse(const QByteArray& data, MavlinkPacket& packet, QString* error)
{
    const QString reason = validationError(data);
    if (error) *error = reason;
    if (!reason.isEmpty()) return false;
    const bool v2 = byteAt(data, 0) == mavlink::MAGIC_V2;
    const int header = v2 ? HEADER_V2 : HEADER_V1;
    packet.sequence = byteAt(data, v2 ? 4 : 2);
    packet.systemId = byteAt(data, v2 ? 5 : 3);
    packet.componentId = byteAt(data, v2 ? 6 : 4);
    packet.messageId = v2 ? byteAt(data, 7) | byteAt(data, 8) << 8 | byteAt(data, 9) << 16 : byteAt(data, 5);
    packet.payload = data.mid(header, byteAt(data, 1));
    return true;
}
QByteArray MavlinkMessageParser::pack(const MavlinkPacket& packet)
{
    // MAVLink 2 отбрасывает нули в конце полезной нагрузки; первый байт остаётся.
    QByteArray payload = packet.payload.left(255);
    while (payload.size() > 1 && payload.endsWith('\0')) payload.chop(1);
    QByteArray data;
    data.append(static_cast<char>(mavlink::MAGIC_V2));
    data.append(static_cast<char>(payload.size()));
    data.append('\0');
    data.append('\0');
    data.append(static_cast<char>(packet.sequence));
    data.append(static_cast<char>(packet.systemId));
    data.append(static_cast<char>(packet.componentId));
    for (int shift = 0; shift < 24; shift += 8) data.append(static_cast<char>((packet.messageId >> shift) & 0xFF));
    data.append(payload);
    quint16 crc = crc16(data.mid(1));
    crc = crc16(QByteArray(1, static_cast<char>(crcExtra(packet.messageId))), crc);
    data.append(static_cast<char>(crc & 0xFF));
    data.append(static_cast<char>(crc >> 8));
    return data;
}

QByteArray MavlinkMessageParser::encode(const Heartbeat& message)
{
    return write([&](QDataStream& stream) {
        stream << message.customMode << message.type << message.autopilot << message.baseMode
               << message.systemStatus << message.mavlinkVersion;
    });
}
QByteArray MavlinkMessageParser::encode(const GlobalPositionInt& message)
{
    return write([&](QDataStream& stream) {
        stream << message.timeBootMs << message.latitudeE7 << message.longitudeE7 << message.altitudeMm
               << message.relativeAltitudeMm << message.vx << message.vy << message.vz << message.heading;
    });
}
QByteArray MavlinkMessageParser::encode(const CommandLong& message)
{
    return write([&](QDataStream& stream) {
        for (const float param : message.params) stream << param;
        stream << message.command << message.targetSystem << message.targetComponent << message.confirmation;
    });
}
QByteArray MavlinkMessageParser::encode(const CommandAck& message)
{
    return write([&](QDataStream& stream) { stream << message.command << message.result; });
}
Heartbeat MavlinkMessageParser::decodeHeartbeat(const QByteArray& payload)
{
    Heartbeat message;
    read(payload, mavlink::HEARTBEAT, [&](QDataStream& stream) {
        stream >> message.customMode >> message.type >> message.autopilot >> message.baseMode
               >> message.systemStatus >> message.mavlinkVersion;
    });
    return message;
}
GlobalPositionInt MavlinkMessageParser::decodeGlobalPosition(const QByteArray& payload)
{
    GlobalPositionInt message;
    read(payload, mavlink::GLOBAL_POSITION_INT, [&](QDataStream& stream) {
        stream >> message.timeBootMs >> message.latitudeE7 >> message.longitudeE7 >> message.altitudeMm
               >> message.relativeAltitudeMm >> message.vx >> message.vy >> message.vz >> message.heading;
    });
    return message;
}
CommandLong MavlinkMessageParser::decodeCommandLong(const QByteArray& payload)
{
    CommandLong message;
    read(payload, mavlink::COMMAND_LONG, [&](QDataStream& stream) {
        for (float& param : message.params) stream >> param;
        stream >> message.command >> message.targetSystem >> message.targetComponent >> message.confirmation;
    });
    return message;
}
CommandAck MavlinkMessageParser::decodeCommandAck(const QByteArray& payload)
{
    CommandAck message;
    read(payload, mavlink::COMMAND_ACK, [&](QDataStream& stream) { stream >> message.command >> message.result; });
    return message;
}

QString MavlinkMessageParser::flightModeName(quint32 customMode)
{
    // Учебная нумерация режимов из PDF; в PX4/ArduPilot custom_mode кодируется иначе.
    switch (customMode) {
    case 0: return "MANUAL";
    case 1: return "STABILIZED";
    case 2: return "ALT_HOLD";
    case 3: return "POS_HOLD";
    case 4: return "GUIDED";
    case 5: return "AUTO";
    }
    return "UNKNOWN";
}
QString MavlinkMessageParser::resultText(quint8 result)
{
    switch (result) {
    case mavlink::MAV_RESULT_ACCEPTED: return "Команда принята";
    case mavlink::MAV_RESULT_TEMPORARILY_REJECTED: return "Команда временно отклонена";
    case mavlink::MAV_RESULT_DENIED: return "Команда отклонена";
    case mavlink::MAV_RESULT_UNSUPPORTED: return "Команда не поддерживается";
    case mavlink::MAV_RESULT_FAILED: return "Ошибка выполнения команды";
    case mavlink::MAV_RESULT_IN_PROGRESS: return "Команда выполняется";
    }
    return "Неизвестный результат";
}
QString MavlinkMessageParser::commandName(int command)
{
    switch (command) {
    case mavlink::MAV_CMD_NAV_TAKEOFF: return "MAV_CMD_NAV_TAKEOFF";
    case mavlink::MAV_CMD_NAV_LAND: return "MAV_CMD_NAV_LAND";
    case mavlink::MAV_CMD_NAV_LOITER_UNLIM: return "MAV_CMD_NAV_LOITER_UNLIM";
    }
    return QString("MAV_CMD %1").arg(command);
}
QString MavlinkMessageParser::stateName(quint8 state)
{
    static const char* names[] = {"UNINIT", "BOOT", "CALIBRATING", "STANDBY", "ACTIVE",
                                  "CRITICAL", "EMERGENCY", "POWEROFF", "FLIGHT_TERMINATION"};
    return state < 9 ? names[state] : QString("STATE %1").arg(state);
}
