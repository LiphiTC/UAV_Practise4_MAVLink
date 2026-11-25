#pragma once

#include "mavlinkmessageparser.h"
#include <QDateTime>
#include <QHostAddress>
#include <QObject>
#include <array>

class QTimer;
class QUdpSocket;
class TelemetryModel;

// Сетевой слой НСУ: QUdpSocket, приём/отправка MAVLink, контроль связи и COMMAND_ACK.
class MavlinkManager : public QObject {
    Q_OBJECT
public:
    explicit MavlinkManager(TelemetryModel* telemetry, QObject* parent = nullptr);
    bool start(quint16 listenPort);
    void stop();
    bool isListening() const;
    quint16 listenPort() const;
    void setTarget(const QString& address, quint16 port);
    quint64 receivedPackets() const;
    quint64 lostPackets() const;
    quint64 errorPackets() const;
    QDateTime lastPacketTime() const;
    bool isWaitingAck() const;
    static constexpr int linkTimeoutMs = 3000;
    static constexpr int ackTimeoutMs = 3000;
public slots:
    void sendTakeoff(double targetAltitude);
    void sendLand();
    void sendHold();
signals:
    void telemetryReceived(double latitude, double longitude, double altitude, const QString& flightMode);
    void linkStateChanged(bool connected, const QString& address, quint16 port);
    void packetReceived(quint64 received, quint64 lost);
    void packetRejected(quint64 errors, const QString& reason);
    void vehicleDetected(int systemId, int componentId);
    void vehicleStatusChanged(bool healthy, const QString& status);
    void commandSent(int command, double parameter);
    void commandResult(int command, bool accepted, const QString& message);
    void errorOccurred(const QString& message);
private slots:
    void onReadyRead();
    void checkLinkTimeout();
    void onAckTimeout();
private:
    void processDatagram(const QByteArray& data, const QHostAddress& sender, quint16 senderPort);
    void handleHeartbeat(const MavlinkPacket& packet, const QHostAddress& sender, quint16 senderPort);
    void handleCommandAck(const CommandAck& ack);
    void sendCommand(quint16 command, const std::array<float, 7>& params, double parameter);
    QUdpSocket* m_udpSocket = nullptr;
    TelemetryModel* m_telemetry = nullptr;
    QTimer* m_linkTimer = nullptr;
    QTimer* m_ackTimer = nullptr;
    QHostAddress m_targetAddress;
    quint16 m_targetPort = 0;
    QString m_lastAddress;
    quint16 m_lastPort = 0;
    QDateTime m_lastPacketTime;
    bool m_linkConnected = false;
    quint64 m_receivedPackets = 0;
    quint64 m_lostPackets = 0;
    quint64 m_errorPackets = 0;
    qint64 m_lastSequence = -1;
    // НСУ — sysid 255, MAV_COMP_ID_MISSIONPLANNER; цель берётся из HEARTBEAT.
    quint8 m_systemId = 255;
    quint8 m_componentId = 190;
    quint8 m_targetSystemId = 1;
    quint8 m_targetComponentId = 1;
    quint8 m_sequence = 0;
    int m_pendingCommand = -1;
    int m_vehicleKey = -1;
    QString m_flightMode = "UNKNOWN";
    int m_systemStatus = -1;
};
