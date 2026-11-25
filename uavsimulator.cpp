#include "uavsimulator.h"
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QTimer>
#include <QUdpSocket>
#include <algorithm>
#include <cmath>

namespace {
constexpr double homeLatitude = 55.752220;
constexpr double homeLongitude = 37.615560;
constexpr double homeAltitudeAmsl = 150.0;
constexpr double metersPerDegree = 111320.0;
constexpr double pi = 3.141592653589793;
constexpr double orbitRadius = 60.0;
constexpr double orbitSpeed = 6.0;
constexpr double climbRate = 5.0;
constexpr double landingRate = 3.0;
}

UavSimulator::UavSimulator(QObject* parent) : QObject(parent)
{
    m_telemetryTimer = new QTimer(this);
    m_telemetryTimer->setObjectName("telemetryTimer");
    m_telemetryTimer->setInterval(telemetryIntervalMs);
    connect(m_telemetryTimer, &QTimer::timeout, this, &UavSimulator::sendTelemetry);
    m_bootTime.start();
}
bool UavSimulator::isRunning() const { return m_udpSocket != nullptr; }
void UavSimulator::setLocalPort(quint16 port) { m_localPort = port; }
quint16 UavSimulator::localPort() const { return m_udpSocket ? m_udpSocket->localPort() : m_localPort; }
void UavSimulator::setSystemId(quint8 systemId) { m_systemId = systemId; }
double UavSimulator::altitude() const { return m_altitude; }
QString UavSimulator::flightMode() const { return MavlinkMessageParser::flightModeName(m_flightMode); }
void UavSimulator::corruptNextPacket() { m_corruptNextPacket = true; }
void UavSimulator::dropNextPackets(int count) { m_dropPackets = std::max(0, count); }
void UavSimulator::setSensorFailure(bool failed) { m_sensorFailure = failed; }
void UavSimulator::setAckEnabled(bool enabled) { m_ackEnabled = enabled; }
bool UavSimulator::start(const QString& targetAddress, quint16 targetPort)
{
    stop();
    if (QHostAddress(targetAddress).isNull() || targetPort == 0) {
        emit errorOccurred(QString("Неверный адрес НСУ: %1:%2").arg(targetAddress).arg(targetPort));
        return false;
    }
    m_targetAddress = targetAddress;
    m_targetPort = targetPort;
    m_udpSocket = new QUdpSocket(this);
    // Сокет привязан, чтобы принимать COMMAND_LONG от НСУ.
    if (!m_udpSocket->bind(QHostAddress::AnyIPv4, m_localPort, QUdpSocket::DontShareAddress)) {
        emit errorOccurred(QString("Эмулятор не открыл UDP-порт %1: %2").arg(m_localPort).arg(m_udpSocket->errorString()));
        delete m_udpSocket;
        m_udpSocket = nullptr;
        return false;
    }
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &UavSimulator::onReadyRead);
    m_tick = 0;
    m_telemetryTimer->start();
    emit runningChanged(true);
    return true;
}
void UavSimulator::stop()
{
    if (!m_udpSocket) return;
    m_telemetryTimer->stop();
    m_udpSocket->close();
    m_udpSocket->deleteLater();
    m_udpSocket = nullptr;
    emit runningChanged(false);
}
void UavSimulator::advance(double dt)
{
    switch (m_flightMode) {
    case Guided:
        m_verticalSpeed = std::clamp((m_targetAltitude - m_altitude) / dt, -climbRate, climbRate);
        break;
    case Auto:
        m_verticalSpeed = -std::min(landingRate, m_altitude / dt);
        break;
    default:
        m_verticalSpeed = 0;
        break;
    }
    m_altitude = std::max(0.0, m_altitude + m_verticalSpeed * dt);
    if (m_flightMode == Auto && m_altitude <= 0.0) {
        m_flightMode = Stabilized;
        m_targetAltitude = 0;
    }
    // В режиме GUIDED аппарат облетает дом по кругу, в остальных висит на месте.
    if (m_flightMode == Guided && m_altitude > 0.5)
        m_phase = std::fmod(m_phase + orbitSpeed / orbitRadius * dt, 2 * pi);
    const double north = orbitRadius * std::sin(m_phase);
    const double east = orbitRadius * (std::cos(m_phase) - 1);
    m_latitude = homeLatitude + north / metersPerDegree;
    m_longitude = homeLongitude + east / (metersPerDegree * std::cos(homeLatitude * pi / 180));
}
void UavSimulator::sendTelemetry()
{
    if (!m_udpSocket) return;
    const double dt = telemetryIntervalMs / 1000.0;
    advance(dt);
    if (m_tick++ % 10 == 0) sendHeartbeat();
    const bool orbiting = m_flightMode == Guided && m_altitude > 0.5;
    const double north = orbiting ? orbitSpeed * std::cos(m_phase) : 0;
    const double east = orbiting ? -orbitSpeed * std::sin(m_phase) : 0;
    GlobalPositionInt position;
    position.timeBootMs = static_cast<quint32>(m_bootTime.elapsed() % 4294967296LL);
    position.latitudeE7 = static_cast<qint32>(std::lround(m_latitude * 1e7));
    position.longitudeE7 = static_cast<qint32>(std::lround(m_longitude * 1e7));
    position.altitudeMm = static_cast<qint32>(std::lround((homeAltitudeAmsl + m_altitude) * 1000.0));
    position.relativeAltitudeMm = static_cast<qint32>(std::lround(m_altitude * 1000.0));
    position.vx = static_cast<qint16>(std::lround(north * 100));
    position.vy = static_cast<qint16>(std::lround(east * 100));
    position.vz = static_cast<qint16>(std::lround(-m_verticalSpeed * 100));
    const double heading = std::fmod(std::atan2(east, north) * 180 / pi + 360, 360);
    position.heading = static_cast<quint16>(std::lround(heading * 100) % 36000);
    sendMessage(mavlink::GLOBAL_POSITION_INT, MavlinkMessageParser::encode(position),
                QHostAddress(m_targetAddress), m_targetPort);
}
void UavSimulator::sendHeartbeat()
{
    const bool airborne = m_altitude > 0.05;
    Heartbeat heartbeat;
    heartbeat.customMode = m_flightMode;
    heartbeat.type = 2;        // MAV_TYPE_QUADROTOR
    heartbeat.autopilot = 0;   // MAV_AUTOPILOT_GENERIC
    // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, в воздухе также MAV_MODE_FLAG_SAFETY_ARMED.
    heartbeat.baseMode = airborne ? 0x81 : 0x01;
    heartbeat.systemStatus = m_sensorFailure ? mavlink::MAV_STATE_CRITICAL
                             : airborne ? mavlink::MAV_STATE_ACTIVE : mavlink::MAV_STATE_STANDBY;
    sendMessage(mavlink::HEARTBEAT, MavlinkMessageParser::encode(heartbeat), QHostAddress(m_targetAddress), m_targetPort);
}
void UavSimulator::sendMessage(quint32 messageId, const QByteArray& payload, const QHostAddress& address, quint16 port)
{
    MavlinkPacket packet;
    packet.sequence = m_sequence++;
    packet.systemId = m_systemId;
    packet.componentId = m_componentId;
    packet.messageId = messageId;
    packet.payload = payload;
    // Номер seq уже израсходован: НСУ увидит пропуск в последовательности.
    if (m_dropPackets > 0) {
        --m_dropPackets;
        return;
    }
    QByteArray datagram = MavlinkMessageParser::pack(packet);
    if (m_corruptNextPacket) {
        datagram[3] = static_cast<char>(static_cast<quint8>(datagram[3]) ^ 0xFF);
        m_corruptNextPacket = false;
    }
    if (m_udpSocket->writeDatagram(datagram, address, port) == -1)
        emit errorOccurred("Эмулятор: ошибка отправки — " + m_udpSocket->errorString());
}
void UavSimulator::onReadyRead()
{
    while (m_udpSocket && m_udpSocket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        MavlinkPacket packet;
        if (!datagram.isValid() || !MavlinkMessageParser::parse(datagram.data(), packet)) continue;
        if (packet.messageId == mavlink::COMMAND_LONG)
            handleCommand(MavlinkMessageParser::decodeCommandLong(packet.payload), datagram.senderAddress(),
                          static_cast<quint16>(datagram.senderPort()));
    }
}
void UavSimulator::handleCommand(const CommandLong& command, const QHostAddress& sender, quint16 senderPort)
{
    if (command.targetSystem != 0 && command.targetSystem != m_systemId) return;
    // Режим «нет ответа»: команда будто не дошла, COMMAND_ACK не отправляется.
    if (!m_ackEnabled) {
        emit commandReceived(command.command, -1);
        return;
    }
    const bool airborne = m_altitude > 0.05;
    quint8 result = mavlink::MAV_RESULT_UNSUPPORTED;
    switch (command.command) {
    case mavlink::MAV_CMD_NAV_TAKEOFF: {
        const double altitude = command.params[6];
        if (m_sensorFailure || !std::isfinite(altitude) || altitude <= 0 || altitude > 500) {
            result = mavlink::MAV_RESULT_DENIED;
        } else {
            m_targetAltitude = altitude;
            m_flightMode = Guided;
            result = mavlink::MAV_RESULT_ACCEPTED;
        }
        break;
    }
    case mavlink::MAV_CMD_NAV_LAND:
        result = airborne ? mavlink::MAV_RESULT_ACCEPTED : mavlink::MAV_RESULT_DENIED;
        if (airborne) m_flightMode = Auto;
        break;
    case mavlink::MAV_CMD_NAV_LOITER_UNLIM:
        result = airborne ? mavlink::MAV_RESULT_ACCEPTED : mavlink::MAV_RESULT_DENIED;
        if (airborne) m_flightMode = PosHold;
        break;
    default:
        break;
    }
    CommandAck ack;
    ack.command = command.command;
    ack.result = result;
    sendMessage(mavlink::COMMAND_ACK, MavlinkMessageParser::encode(ack), sender, senderPort);
    emit commandReceived(command.command, result);
}
