#include "mavlinkmanager.h"
#include "telemetrymodel.h"
#include "uavsimulator.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QNetworkDatagram>
#include <QSignalSpy>
#include <QTest>
#include <QUdpSocket>
#include <cmath>
#include <limits>

namespace {
void check(bool condition, const char* message) { if (!condition) qFatal("FAILED: %s", message); }
QByteArray frame(quint8 sequence, quint32 messageId, const QByteArray& payload)
{
    MavlinkPacket packet;
    packet.sequence = sequence;
    packet.systemId = 1;
    packet.componentId = 1;
    packet.messageId = messageId;
    packet.payload = payload;
    return MavlinkMessageParser::pack(packet);
}
quint8 byteAt(const QByteArray& data, int index) { return static_cast<quint8>(data.at(index)); }
}
int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    using Parser = MavlinkMessageParser;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    // Формат MAVLink 2, CRC и кодирование сообщений.
    check(Parser::crc16("123456789") == 0x6F91, "CRC-16/MCRF4XX check value");
    check(Parser::crcExtra(mavlink::HEARTBEAT) == 50 && Parser::crcExtra(mavlink::GLOBAL_POSITION_INT) == 104
        && Parser::crcExtra(mavlink::COMMAND_LONG) == 152 && Parser::crcExtra(mavlink::COMMAND_ACK) == 143,
        "CRC_EXTRA from common.xml");
    Heartbeat heartbeat;
    heartbeat.customMode = 4;
    heartbeat.baseMode = 0x81;
    heartbeat.systemStatus = mavlink::MAV_STATE_ACTIVE;
    const QByteArray heartbeatFrame = frame(10, mavlink::HEARTBEAT, Parser::encode(heartbeat));
    check(heartbeatFrame.size() == 21 && byteAt(heartbeatFrame, 0) == 0xFD && byteAt(heartbeatFrame, 1) == 9
        && byteAt(heartbeatFrame, 4) == 10 && byteAt(heartbeatFrame, 5) == 1, "HEARTBEAT frame layout");
    MavlinkPacket packet;
    QString error;
    check(Parser::parse(heartbeatFrame, packet, &error) && error.isEmpty() && packet.sequence == 10
        && packet.messageId == mavlink::HEARTBEAT, "parse HEARTBEAT");
    const Heartbeat decodedHeartbeat = Parser::decodeHeartbeat(packet.payload);
    check(decodedHeartbeat.customMode == 4 && decodedHeartbeat.baseMode == 0x81 && decodedHeartbeat.mavlinkVersion == 3
        && decodedHeartbeat.systemStatus == mavlink::MAV_STATE_ACTIVE, "HEARTBEAT fields");
    GlobalPositionInt position;
    position.timeBootMs = 123456;
    position.latitudeE7 = 557522200;
    position.longitudeE7 = 376155600;
    position.altitudeMm = 192500;
    position.relativeAltitudeMm = 42500;
    position.vz = -500;
    position.heading = 9000;
    const QByteArray positionFrame = frame(13, mavlink::GLOBAL_POSITION_INT, Parser::encode(position));
    check(Parser::parse(positionFrame, packet), "parse GLOBAL_POSITION_INT");
    const GlobalPositionInt decodedPosition = Parser::decodeGlobalPosition(packet.payload);
    check(decodedPosition.latitudeE7 == 557522200 && decodedPosition.longitudeE7 == 376155600
        && decodedPosition.relativeAltitudeMm == 42500 && decodedPosition.vz == -500
        && decodedPosition.heading == 9000 && decodedPosition.timeBootMs == 123456, "GLOBAL_POSITION_INT fields");
    CommandLong takeoff;
    takeoff.params = {0.0f, 0.0f, 0.0f, nan, nan, nan, 80.0f};
    takeoff.command = mavlink::MAV_CMD_NAV_TAKEOFF;
    takeoff.targetSystem = 1;
    takeoff.targetComponent = 1;
    check(Parser::encode(takeoff).size() == 33, "COMMAND_LONG payload is 33 bytes");
    check(Parser::parse(frame(0, mavlink::COMMAND_LONG, Parser::encode(takeoff)), packet), "parse COMMAND_LONG");
    const CommandLong decodedCommand = Parser::decodeCommandLong(packet.payload);
    check(decodedCommand.command == 22 && decodedCommand.params[6] == 80.0f && std::isnan(decodedCommand.params[3])
        && decodedCommand.targetSystem == 1, "TAKEOFF: param7 altitude, NaN for current position");
    CommandAck ack;
    ack.command = mavlink::MAV_CMD_NAV_TAKEOFF;
    ack.result = mavlink::MAV_RESULT_ACCEPTED;
    const QByteArray ackFrame = frame(1, mavlink::COMMAND_ACK, Parser::encode(ack));
    check(byteAt(ackFrame, 1) == 1 && ackFrame.size() == 13, "MAVLink 2 truncates trailing zero bytes");
    check(Parser::parse(ackFrame, packet), "parse truncated COMMAND_ACK");
    const CommandAck decodedAck = Parser::decodeCommandAck(packet.payload);
    check(decodedAck.command == 22 && decodedAck.result == mavlink::MAV_RESULT_ACCEPTED, "truncated payload zero-filled");

    // Повреждённые пакеты: длина, стартовый байт, CRC.
    QByteArray corrupted = heartbeatFrame;
    corrupted[3] = static_cast<char>(byteAt(corrupted, 3) ^ 0xFF);
    check(!Parser::isValid(corrupted) && Parser::validationError(corrupted).contains("контрольная сумма"),
        "flipped compat flags fail CRC");
    QByteArray payloadBit = positionFrame;
    payloadBit[15] = static_cast<char>(byteAt(payloadBit, 15) ^ 0x01);
    check(Parser::validationError(payloadBit).contains("контрольная сумма"), "one payload bit fails CRC");
    check(Parser::validationError(heartbeatFrame.left(11)).contains("короткий"), "short packet rejected");
    QByteArray badMagic = heartbeatFrame;
    badMagic[0] = 0x55;
    check(Parser::validationError(badMagic).contains("стартовый"), "bad magic rejected");
    check(Parser::validationError(heartbeatFrame.left(heartbeatFrame.size() - 1)).contains("Длина"), "length mismatch rejected");
    check(Parser::validationError(frame(0, 30, QByteArray(28, '\1'))).contains("Неподдерживаемое"), "unknown message rejected");
    QByteArray badFlags = heartbeatFrame;
    badFlags[2] = 0x02;
    check(Parser::validationError(badFlags).contains("флаги"), "unknown incompat flags rejected");
    // MAVLink 1: FE len seq sysid compid msgid payload crc.
    QByteArray v1 = QByteArray::fromHex("0905010100") + Parser::encode(heartbeat);
    quint16 crc = Parser::crc16(v1);
    crc = Parser::crc16(QByteArray(1, char(50)), crc);
    v1.prepend(static_cast<char>(0xFE));
    v1.append(static_cast<char>(crc & 0xFF));
    v1.append(static_cast<char>(crc >> 8));
    check(Parser::parse(v1, packet) && packet.sequence == 5 && packet.messageId == mavlink::HEARTBEAT, "MAVLink 1 frame accepted");
    check(Parser::flightModeName(4) == "GUIDED" && Parser::flightModeName(42) == "UNKNOWN"
        && Parser::resultText(mavlink::MAV_RESULT_DENIED) == "Команда отклонена"
        && Parser::stateName(mavlink::MAV_STATE_CRITICAL) == "CRITICAL", "names from the assignment");

    // Модель телеметрии.
    TelemetryModel model;
    QSignalSpy modelTelemetry(&model, &TelemetryModel::telemetryChanged);
    QSignalSpy modelLink(&model, &TelemetryModel::linkStateChanged);
    QSignalSpy modelMode(&model, &TelemetryModel::flightModeChanged);
    model.updateTelemetry(55.75222, 37.61556, 10, "GUIDED");
    check(modelTelemetry.size() == 1 && model.lastUpdate().isValid() && modelMode.size() == 1, "telemetry update");
    model.updateTelemetry(std::nan(""), 37.6, 10, "AUTO");
    model.updateTelemetry(91, 37.6, 10, "AUTO");
    check(modelTelemetry.size() == 1 && model.flightMode() == "GUIDED", "invalid telemetry rejected");
    model.updateTelemetry(55.75222 + 100 / 111320.0, 37.61556, 20, "GUIDED");
    check(std::abs(model.localPosition().y - 100) < 0.01 && std::abs(model.localPosition().x) < 0.01
        && model.localPosition().z == 20 && modelMode.size() == 1, "local position from home");
    model.updateLink(true, "127.0.0.1", 14551);
    model.updateLink(true, "127.0.0.1", 14551);
    check(modelLink.size() == 1 && model.isLinkConnected(), "link signal only on change");
    model.updateLink(false, QString(), 0);
    model.updateLink(false, QString(), 0);
    check(modelLink.size() == 2 && !model.isLinkConnected() && model.lastAddress() == "127.0.0.1", "link lost keeps last sender");

    // MavlinkManager: приём из сырого сокета, потери seq, повреждения, тайм-аут связи.
    TelemetryModel telemetry;
    MavlinkManager manager(&telemetry);
    QObject::connect(&manager, &MavlinkManager::telemetryReceived, &telemetry, &TelemetryModel::updateTelemetry);
    QObject::connect(&manager, &MavlinkManager::linkStateChanged, &telemetry, &TelemetryModel::updateLink);
    QObject::connect(&manager, &MavlinkManager::vehicleDetected, &telemetry, &TelemetryModel::updateVehicle);
    QSignalSpy errors(&manager, &MavlinkManager::errorOccurred);
    manager.sendTakeoff(80);
    check(errors.size() == 1 && errors.last()[0].toString().contains("закрыт"), "no command without UDP port");
    check(manager.start(0) && manager.isListening() && manager.listenPort() != 0, "bind UDP port");
    const quint16 port = manager.listenPort();
    MavlinkManager busy(&telemetry);
    QSignalSpy busyErrors(&busy, &MavlinkManager::errorOccurred);
    check(!busy.start(port) && busyErrors.last()[0].toString().contains(QString::number(port)), "busy port reported");
    QSignalSpy received(&manager, &MavlinkManager::packetReceived);
    QSignalSpy rejected(&manager, &MavlinkManager::packetRejected);
    QSignalSpy link(&manager, &MavlinkManager::linkStateChanged);
    QSignalSpy positions(&manager, &MavlinkManager::telemetryReceived);
    QUdpSocket raw;
    check(raw.bind(QHostAddress::LocalHost, 0), "raw socket");
    auto send = [&](const QByteArray& data) { raw.writeDatagram(data, QHostAddress::LocalHost, port); };
    send(heartbeatFrame);
    check(QTest::qWaitFor([&] { return received.size() == 1; }, 2000), "HEARTBEAT received");
    check(link.size() == 1 && link.last()[0].toBool() && link.last()[1].toString() == "127.0.0.1"
        && link.last()[2].toUInt() == raw.localPort(), "sender address and port from QNetworkDatagram");
    check(telemetry.isLinkConnected() && telemetry.systemId() == 1 && telemetry.componentId() == 1, "vehicle sysid:compid");
    send(positionFrame);
    check(QTest::qWaitFor([&] { return positions.size() == 1; }, 2000), "GLOBAL_POSITION_INT received");
    check(manager.lostPackets() == 2, "seq 10 -> 13: packets 11 and 12 lost");
    check(std::abs(positions.last()[0].toDouble() - 55.75222) < 1e-9 && positions.last()[2].toDouble() == 42.5
        && positions.last()[3].toString() == "GUIDED", "E7 / mm conversion, mode from HEARTBEAT");
    QByteArray badPosition = frame(14, mavlink::GLOBAL_POSITION_INT, Parser::encode(position));
    badPosition[3] = static_cast<char>(byteAt(badPosition, 3) ^ 0xFF);
    send(badPosition);
    check(QTest::qWaitFor([&] { return rejected.size() == 1; }, 2000), "corrupted packet detected");
    check(manager.errorPackets() == 1 && manager.receivedPackets() == 2 && positions.size() == 1
        && rejected.last()[1].toString().contains("контрольная сумма"), "corrupted data not applied");
    send(frame(15, mavlink::GLOBAL_POSITION_INT, Parser::encode(position)));
    check(QTest::qWaitFor([&] { return positions.size() == 2; }, 2000), "work continues after corrupted packet");
    check(manager.lostPackets() == 3, "corrupted seq 14 is also a gap");
    send(frame(16, mavlink::COMMAND_ACK, Parser::encode(ack)));
    check(QTest::qWaitFor([&] { return errors.size() == 2; }, 2000)
        && errors.last()[0].toString().contains("без ожидающей"), "unexpected COMMAND_ACK reported");
    QElapsedTimer silence;
    silence.start();
    check(QTest::qWaitFor([&] { return !telemetry.isLinkConnected(); }, 5000), "link timeout");
    check(silence.elapsed() >= 2900 && errors.last()[0].toString() == "Связь с БПЛА потеряна"
        && !link.last()[0].toBool(), "link lost after 3 s without packets");
    manager.sendTakeoff(80);
    check(errors.last()[0].toString() == "Невозможно отправить команду: нет связи", "command blocked without link");

    // MavlinkManager + UavSimulator: телеметрия, команда, ACK, отказы.
    UavSimulator uav;
    uav.setLocalPort(0);
    QSignalSpy uavCommands(&uav, &UavSimulator::commandReceived);
    QSignalSpy running(&uav, &UavSimulator::runningChanged);
    check(!uav.start("not an address", port), "invalid GCS address rejected");
    check(uav.start("127.0.0.1", port) && uav.isRunning() && running.size() == 1, "simulator start");
    check(QTest::qWaitFor([&] { return telemetry.isLinkConnected() && positions.size() > 4; }, 3000), "telemetry at 10 Hz");
    check(telemetry.altitude() == 0 && telemetry.flightMode() == "STABILIZED", "simulator starts on the ground");
    QSignalSpy sent(&manager, &MavlinkManager::commandSent);
    QSignalSpy results(&manager, &MavlinkManager::commandResult);
    manager.sendTakeoff(80);
    check(sent.size() == 1 && sent.last()[0].toInt() == 22 && sent.last()[1].toDouble() == 80
        && manager.isWaitingAck(), "MAV_CMD_NAV_TAKEOFF sent, waiting ACK");
    manager.sendLand();
    check(errors.last()[0].toString().contains("Ожидается COMMAND_ACK") && sent.size() == 1, "no duplicate command before ACK");
    check(QTest::qWaitFor([&] { return results.size() == 1; }, 2000), "COMMAND_ACK received");
    check(results.last()[0].toInt() == 22 && results.last()[1].toBool() && results.last()[2].toString() == "Команда принята"
        && !manager.isWaitingAck() && uavCommands.last()[1].toInt() == mavlink::MAV_RESULT_ACCEPTED, "takeoff accepted");
    check(QTest::qWaitFor([&] { return telemetry.altitude() > 3 && telemetry.flightMode() == "GUIDED"; }, 3000), "UAV climbs");
    const quint64 lostBefore = manager.lostPackets();
    const quint64 receivedBefore = manager.receivedPackets();
    uav.dropNextPackets(2);
    check(QTest::qWaitFor([&] { return manager.receivedPackets() > receivedBefore + 2; }, 3000), "stream continues after gap");
    check(manager.lostPackets() == lostBefore + 2, "two skipped packets counted");
    const quint64 errorsBefore = manager.errorPackets();
    uav.corruptNextPacket();
    check(QTest::qWaitFor([&] { return manager.errorPackets() == errorsBefore + 1; }, 2000), "simulator corruption detected");
    QSignalSpy status(&manager, &MavlinkManager::vehicleStatusChanged);
    uav.setSensorFailure(true);
    check(QTest::qWaitFor([&] { return !status.isEmpty() && !status.last()[0].toBool(); }, 2500)
        && status.last()[1].toString() == "CRITICAL", "sensor failure via HEARTBEAT system_status");
    manager.sendTakeoff(100);
    check(QTest::qWaitFor([&] { return results.size() == 2; }, 2000) && !results.last()[1].toBool()
        && results.last()[2].toString() == "Команда отклонена", "takeoff denied during failure");
    uav.setSensorFailure(false);
    check(QTest::qWaitFor([&] { return status.last()[0].toBool(); }, 2500), "vehicle recovered");
    uav.setAckEnabled(false);
    QElapsedTimer waiting;
    waiting.start();
    manager.sendHold();
    check(QTest::qWaitFor([&] { return results.size() == 3; }, 5000), "ACK timeout fires");
    check(waiting.elapsed() >= 2900 && !results.last()[1].toBool() && results.last()[2].toString() == "Подтверждение не получено"
        && uavCommands.last()[1].toInt() == -1, "no COMMAND_ACK within 3 s");
    uav.setAckEnabled(true);
    manager.sendHold();
    check(QTest::qWaitFor([&] { return results.size() == 4; }, 2000) && results.last()[1].toBool(), "hold accepted");
    check(QTest::qWaitFor([&] { return telemetry.flightMode() == "POS_HOLD"; }, 2000), "hold mode");
    manager.sendLand();
    check(QTest::qWaitFor([&] { return results.size() == 5; }, 2000) && results.last()[1].toBool(), "land accepted");
    check(QTest::qWaitFor([&] { return telemetry.flightMode() == "AUTO"; }, 2000), "landing mode");
    CommandLong unknown;
    unknown.command = 400;
    unknown.targetSystem = 1;
    raw.writeDatagram(frame(0, mavlink::COMMAND_LONG, Parser::encode(unknown)), QHostAddress::LocalHost, uav.localPort());
    check(QTest::qWaitFor([&] { return raw.hasPendingDatagrams(); }, 2000), "simulator answers sender");
    check(Parser::parse(raw.receiveDatagram().data(), packet) && packet.messageId == mavlink::COMMAND_ACK
        && Parser::decodeCommandAck(packet.payload).result == mavlink::MAV_RESULT_UNSUPPORTED, "unsupported command");

    // «Отключить БПЛА» и повторный запуск.
    uav.stop();
    silence.restart();
    check(!uav.isRunning() && running.last()[0].toBool() == false, "simulator stopped");
    check(QTest::qWaitFor([&] { return !telemetry.isLinkConnected(); }, 5000) && silence.elapsed() >= 2900, "link lost after stop");
    const quint64 lostBeforeRestart = manager.lostPackets();
    check(uav.start("127.0.0.1", port), "simulator restart");
    check(QTest::qWaitFor([&] { return telemetry.isLinkConnected(); }, 2000), "link restored");
    QTest::qWait(300);
    check(manager.lostPackets() == lostBeforeRestart && telemetry.flightMode() == "AUTO", "restart continues same flight");
    manager.stop();
    check(!manager.isListening() && !telemetry.isLinkConnected(), "port closed");
    qInfo("All network checks passed");
}
