#include "mavlinkmanager.h"
#include "telemetrymodel.h"
#include <QNetworkDatagram>
#include <QTimer>
#include <QUdpSocket>
#include <cmath>
#include <limits>

MavlinkManager::MavlinkManager(TelemetryModel* telemetry, QObject* parent)
    : QObject(parent), m_telemetry(telemetry)
{
    m_linkTimer = new QTimer(this);
    m_linkTimer->setObjectName("linkTimer");
    m_linkTimer->setInterval(1000);
    connect(m_linkTimer, &QTimer::timeout, this, &MavlinkManager::checkLinkTimeout);
    m_ackTimer = new QTimer(this);
    m_ackTimer->setObjectName("ackTimer");
    m_ackTimer->setSingleShot(true);
    m_ackTimer->setTimerType(Qt::PreciseTimer);
    m_ackTimer->setInterval(ackTimeoutMs);
    connect(m_ackTimer, &QTimer::timeout, this, &MavlinkManager::onAckTimeout);
}
bool MavlinkManager::isListening() const { return m_udpSocket != nullptr; }
quint16 MavlinkManager::listenPort() const { return m_udpSocket ? m_udpSocket->localPort() : 0; }
quint64 MavlinkManager::receivedPackets() const { return m_receivedPackets; }
quint64 MavlinkManager::lostPackets() const { return m_lostPackets; }
quint64 MavlinkManager::errorPackets() const { return m_errorPackets; }
QDateTime MavlinkManager::lastPacketTime() const { return m_lastPacketTime; }
bool MavlinkManager::isWaitingAck() const { return m_pendingCommand >= 0; }
void MavlinkManager::setTarget(const QString& address, quint16 port)
{
    m_targetAddress = QHostAddress(address);
    m_targetPort = port;
}
bool MavlinkManager::start(quint16 listenPort)
{
    stop();
    m_udpSocket = new QUdpSocket(this);
    // DontShareAddress: второй процесс на том же порту получит ошибку, а не половину пакетов.
    if (!m_udpSocket->bind(QHostAddress::AnyIPv4, listenPort, QUdpSocket::DontShareAddress)) {
        emit errorOccurred(QString("Не удалось открыть UDP-порт %1: %2").arg(listenPort).arg(m_udpSocket->errorString()));
        delete m_udpSocket;
        m_udpSocket = nullptr;
        return false;
    }
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &MavlinkManager::onReadyRead);
    m_linkTimer->start();
    emit linkStateChanged(false, QString(), 0);
    return true;
}
void MavlinkManager::stop()
{
    m_linkTimer->stop();
    if (m_pendingCommand >= 0) {
        const int command = m_pendingCommand;
        m_ackTimer->stop();
        m_pendingCommand = -1;
        emit commandResult(command, false, "Канал закрыт до получения подтверждения");
    }
    if (m_udpSocket) {
        m_udpSocket->close();
        m_udpSocket->deleteLater();
        m_udpSocket = nullptr;
    }
    m_lastSequence = -1;
    m_vehicleKey = -1;
    m_systemStatus = -1;
    if (m_linkConnected) {
        m_linkConnected = false;
        emit linkStateChanged(false, m_lastAddress, m_lastPort);
    }
}
void MavlinkManager::onReadyRead()
{
    // Все ожидающие датаграммы за один вызов; сокет может закрыться в обработчике сигнала.
    while (m_udpSocket && m_udpSocket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        if (!datagram.isValid()) continue;
        processDatagram(datagram.data(), datagram.senderAddress(), static_cast<quint16>(datagram.senderPort()));
    }
}
void MavlinkManager::processDatagram(const QByteArray& data, const QHostAddress& sender, quint16 senderPort)
{
    MavlinkPacket packet;
    QString error;
    if (!MavlinkMessageParser::parse(data, packet, &error)) {
        // Повреждённые данные не применяются; следующий корректный пакет обрабатывается как обычно.
        ++m_errorPackets;
        emit packetRejected(m_errorPackets, error);
        return;
    }
    ++m_receivedPackets;
    m_lastPacketTime = QDateTime::currentDateTime();
    // Поле seq позволяет обнаружить пропуски: после 10 пришёл 13 — потеряны 11 и 12.
    if (m_lastSequence >= 0) {
        const qint64 expected = (m_lastSequence + 1) % 256;
        if (packet.sequence != expected) {
            const qint64 lost = (packet.sequence - expected + 256) % 256;
            if (lost > 0 && lost < 100) m_lostPackets += static_cast<quint64>(lost);
        }
    }
    m_lastSequence = packet.sequence;
    const QString address = sender.toString();
    if (!m_linkConnected || address != m_lastAddress || senderPort != m_lastPort) {
        m_linkConnected = true;
        m_lastAddress = address;
        m_lastPort = senderPort;
        emit linkStateChanged(true, m_lastAddress, m_lastPort);
    }
    emit packetReceived(m_receivedPackets, m_lostPackets);
    switch (packet.messageId) {
    case mavlink::HEARTBEAT:
        handleHeartbeat(packet, sender, senderPort);
        break;
    case mavlink::GLOBAL_POSITION_INT: {
        const GlobalPositionInt position = MavlinkMessageParser::decodeGlobalPosition(packet.payload);
        const double latitude = position.latitudeE7 / 1e7;
        const double longitude = position.longitudeE7 / 1e7;
        const double altitude = position.relativeAltitudeMm / 1000.0;
        emit telemetryReceived(latitude, longitude, altitude, m_flightMode);
        break;
    }
    case mavlink::COMMAND_ACK:
        handleCommandAck(MavlinkMessageParser::decodeCommandAck(packet.payload));
        break;
    default:
        break;
    }
}
void MavlinkManager::handleHeartbeat(const MavlinkPacket& packet, const QHostAddress& sender, quint16 senderPort)
{
    const Heartbeat heartbeat = MavlinkMessageParser::decodeHeartbeat(packet.payload);
    const int key = packet.systemId << 8 | packet.componentId;
    if (key != m_vehicleKey) {
        m_vehicleKey = key;
        m_targetSystemId = packet.systemId;
        m_targetComponentId = packet.componentId;
        emit vehicleDetected(packet.systemId, packet.componentId);
    }
    // Команды уходят на адрес, с которого пришёл последний HEARTBEAT.
    m_targetAddress = sender;
    m_targetPort = senderPort;
    m_flightMode = MavlinkMessageParser::flightModeName(heartbeat.customMode);
    if (heartbeat.systemStatus != m_systemStatus) {
        m_systemStatus = heartbeat.systemStatus;
        const bool healthy = heartbeat.systemStatus < mavlink::MAV_STATE_CRITICAL;
        emit vehicleStatusChanged(healthy, MavlinkMessageParser::stateName(heartbeat.systemStatus));
    }
}
void MavlinkManager::checkLinkTimeout()
{
    if (!m_lastPacketTime.isValid()) return;
    const qint64 elapsedMs = m_lastPacketTime.msecsTo(QDateTime::currentDateTime());
    if (elapsedMs > linkTimeoutMs && m_linkConnected) {
        m_linkConnected = false;
        // После восстановления связи нумерация seq может начаться заново.
        m_lastSequence = -1;
        m_vehicleKey = -1;
        m_systemStatus = -1;
        emit linkStateChanged(false, m_lastAddress, m_lastPort);
        emit errorOccurred("Связь с БПЛА потеряна");
    }
}

void MavlinkManager::sendTakeoff(double targetAltitude)
{
    if (!std::isfinite(targetAltitude) || targetAltitude <= 0) {
        emit errorOccurred("Неверная высота взлёта");
        return;
    }
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // param4–6 = NaN: автопилот применяет текущие курс, широту и долготу; param7 — высота, м.
    sendCommand(mavlink::MAV_CMD_NAV_TAKEOFF, {0.0f, 0.0f, 0.0f, nan, nan, nan, static_cast<float>(targetAltitude)},
                targetAltitude);
}
void MavlinkManager::sendLand()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sendCommand(mavlink::MAV_CMD_NAV_LAND, {0.0f, 0.0f, 0.0f, nan, nan, nan, 0.0f}, 0);
}
void MavlinkManager::sendHold()
{
    // Зависание на текущей точке и высоте.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sendCommand(mavlink::MAV_CMD_NAV_LOITER_UNLIM, {0.0f, 0.0f, 0.0f, nan, nan, nan, nan}, 0);
}
void MavlinkManager::sendCommand(quint16 command, const std::array<float, 7>& params, double parameter)
{
    const QString name = MavlinkMessageParser::commandName(command);
    if (!m_udpSocket) {
        emit errorOccurred("UDP-порт НСУ закрыт");
        return;
    }
    if (m_targetAddress.isNull() || m_targetPort == 0) {
        emit errorOccurred("Адрес БПЛА не задан");
        return;
    }
    if (!m_telemetry || !m_telemetry->isLinkConnected()) {
        emit errorOccurred("Невозможно отправить команду: нет связи");
        return;
    }
    // Не дублируем команды, пока не получен ответ или не истёк тайм-аут.
    if (m_pendingCommand >= 0) {
        emit errorOccurred("Ожидается COMMAND_ACK для " + MavlinkMessageParser::commandName(m_pendingCommand));
        return;
    }
    CommandLong message;
    message.params = params;
    message.command = command;
    message.targetSystem = m_targetSystemId;
    message.targetComponent = m_targetComponentId;
    message.confirmation = 0;
    MavlinkPacket packet;
    packet.sequence = m_sequence++;
    packet.systemId = m_systemId;
    packet.componentId = m_componentId;
    packet.messageId = mavlink::COMMAND_LONG;
    packet.payload = MavlinkMessageParser::encode(message);
    const QByteArray datagram = MavlinkMessageParser::pack(packet);
    // Успешный writeDatagram() — передача сетевому стеку, а не доставка до БПЛА.
    const qint64 sent = m_udpSocket->writeDatagram(datagram, m_targetAddress, m_targetPort);
    if (sent == -1) {
        emit errorOccurred("Ошибка отправки " + name + ": " + m_udpSocket->errorString());
        return;
    }
    m_pendingCommand = command;
    m_ackTimer->start();
    emit commandSent(command, parameter);
}
void MavlinkManager::handleCommandAck(const CommandAck& ack)
{
    if (ack.command != m_pendingCommand) {
        emit errorOccurred("COMMAND_ACK без ожидающей команды: " + MavlinkMessageParser::commandName(ack.command));
        return;
    }
    m_ackTimer->stop();
    m_pendingCommand = -1;
    const QString resultText = MavlinkMessageParser::resultText(ack.result);
    const bool accepted = ack.result == mavlink::MAV_RESULT_ACCEPTED || ack.result == mavlink::MAV_RESULT_IN_PROGRESS;
    emit commandResult(static_cast<int>(ack.command), accepted, resultText);
}
void MavlinkManager::onAckTimeout()
{
    if (m_pendingCommand < 0) return;
    const int command = m_pendingCommand;
    m_pendingCommand = -1;
    emit commandResult(command, false, "Подтверждение не получено");
}
