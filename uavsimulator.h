#pragma once

#include "mavlinkmessageparser.h"
#include <QElapsedTimer>
#include <QObject>

class QHostAddress;
class QTimer;
class QUdpSocket;

// Эмулятор БПЛА: телеметрия 10 Гц, HEARTBEAT 1 Гц, ответы COMMAND_ACK.
// Используется и окном НСУ, и отдельным приложением Practical_04_UavEmulator.
class UavSimulator : public QObject {
    Q_OBJECT
public:
    explicit UavSimulator(QObject* parent = nullptr);
    // Локальный порт 0 — любой свободный. Повторный start() продолжает прежний полёт.
    bool start(const QString& targetAddress, quint16 targetPort);
    void stop();
    bool isRunning() const;
    void setLocalPort(quint16 port);
    quint16 localPort() const;
    void setSystemId(quint8 systemId);
    double altitude() const;
    QString flightMode() const;
    // Имитация отказов (задание 4).
    void corruptNextPacket();
    void dropNextPackets(int count);
    void setSensorFailure(bool failed);
    void setAckEnabled(bool enabled);
    static constexpr int telemetryIntervalMs = 100;
public slots:
    void sendTelemetry();
signals:
    void runningChanged(bool running);
    // result = -1: команда получена, но COMMAND_ACK намеренно не отправлен.
    void commandReceived(int command, int result);
    void errorOccurred(const QString& message);
private slots:
    void onReadyRead();
private:
    enum Mode : quint16 { Manual, Stabilized, AltHold, PosHold, Guided, Auto };
    void advance(double dt);
    void sendHeartbeat();
    void sendMessage(quint32 messageId, const QByteArray& payload, const QHostAddress& address, quint16 port);
    void handleCommand(const CommandLong& command, const QHostAddress& sender, quint16 senderPort);
    QUdpSocket* m_udpSocket = nullptr;
    QTimer* m_telemetryTimer = nullptr;
    QElapsedTimer m_bootTime;
    QString m_targetAddress;
    quint16 m_targetPort = 0;
    quint16 m_localPort = 14551;
    quint8 m_systemId = 1;
    quint8 m_componentId = 1;
    double m_altitude = 0.0;
    double m_targetAltitude = 0.0;
    double m_verticalSpeed = 0.0;
    double m_latitude = 55.752220;
    double m_longitude = 37.615560;
    double m_phase = 0.0;
    quint16 m_flightMode = Stabilized;
    quint8 m_sequence = 0;
    int m_tick = 0;
    int m_dropPackets = 0;
    bool m_corruptNextPacket = false;
    bool m_sensorFailure = false;
    bool m_ackEnabled = true;
};
